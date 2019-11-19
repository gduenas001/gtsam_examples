
#include "post_process.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>


#include <vector> 
#include <algorithm> 
#include <typeinfo>

using namespace std;
using namespace gtsam;

typedef std::vector< std::pair<int, double> > pair_vector;


void post_process(
          Values result_fl,
				  FixedLagSmoother::Result isam_result_fl, // TODO: is this used?
          IncrementalFixedLagSmoother &fixed_lag_smoother,
				  map<string, vector<int>> A_rows_per_type,
          Counters &counters,
          Params &params){

 // get the factor graph & Jacobian from isam
  NonlinearFactorGraph factor_graph= fixed_lag_smoother.getFactors();

  // get the linear graph
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph= factor_graph.linearize(result_fl);

  // --------------- print factors --------------------
  {
    int factor_count= -1;
    for (auto factor : factor_graph){
      ++factor_count;
      if (!factor) {continue;}

      double factor_error= 2 * factor->error(result_fl);
      double factor_dim= factor->dim();
      
      cout<< "factor # "<< factor_count<< "\t"
          << "type: "<< counters.types[factor_count]<< "\t\t"
          << "dim: "<< factor_dim<< "\t"
          << "error: "<< 2 * factor_error<< "\t";
      factor->printKeys();
    }

    cout<< "----------------------"<< endl;
  }
  // -----------------------------------


 
  Matrix hessian= (lin_graph->hessian()).first;
  Matrix A= (lin_graph->jacobian()).first;
  Matrix P= hessian.inverse();
  Matrix S = P * A.transpose();
  Matrix S_transpose= S.transpose();

  // rank of Jacobian A
  Eigen::FullPivLU<Matrix> A_lu(A);
  A_lu.setThreshold(1e-7);
  double A_rank= A_lu.rank();
  
  // number of measurements and states
  cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;
  cout<< "rank of A: "<< A_rank<< endl;
  cout<< "Hessian (Lambda) matrix size = "<< hessian.rows()<< " x "<< hessian.cols()<< endl;

  // get variances for the last state
  map<string,double> var= get_variances_for_last_pose(fixed_lag_smoother, counters);
  cout<< "std dev. (roll, pitch, yaw, x, y, z): \n"<< "("<< 
          sqrt(var["roll"])<< ", "<< 
          sqrt(var["pitch"])<< ", "<< 
          sqrt(var["yaw"])<< ", "<< 
          sqrt(var["x"])<< ", "<< 
          sqrt(var["y"])<< ", "<< 
          sqrt(var["z"])<< ")"<< endl;
  
  // builds a map for vector t for each coordinate TODO: change to lat, long, vert (needs rotations)
  map<string, Vector> t_vector= buildt_vector(A.cols());
  
  // upper bound lambda
  double effective_n= get_dof_from_graph(factor_graph, counters);
  double chi_squared_dof= effective_n - A.cols();
  double r= 2 * factor_graph.error(result_fl);
  boost::math::chi_squared_distribution<> chi2_dist_lambda(chi_squared_dof);
  double lambda= pow( sqrt(r) + sqrt(boost::math::quantile(chi2_dist_lambda, 1-1e-5)), 2 );
  cout<< "effective number of measurements: "<< effective_n<< endl;
  cout<< "DOF of the chi-squared: "<< chi_squared_dof<< endl;
  cout<< "r: "<< r<< endl;
  cout<< "Non-centrality parameter upper bound, lambda: "<< lambda<< endl;



  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  
  // check matrix M before elimination
  Matrix M = (Eigen::MatrixXd::Identity(A.rows(), A.rows()) - A*S);
  Eigen::FullPivLU<Matrix> M_lu(M);
  M_lu.setThreshold(1e-7);
  cout<< "size of M = "<< M.rows() << " x "<< M.cols()<< endl;
  cout << "rank of M is " << M_lu.rank() << endl;

  // compute integrity
  boost::math::chi_squared_distribution<> chi2_dist_raim(1);
  map<string, double> h_LIR;

  h_LIR["x"]= 1 - boost::math::cdf(chi2_dist_raim, 
                      pow(params.AL_x / sqrt(var["x"]), 2) );
  h_LIR["y"]= 1 - boost::math::cdf(chi2_dist_raim, 
                      pow(params.AL_y / sqrt(var["y"]), 2) );
  h_LIR["z"]= 1 - boost::math::cdf(chi2_dist_raim, 
                      pow(params.AL_z / sqrt(var["z"]), 2) );

  cout<< "LIR for h in x: "<< h_LIR["x"]<< endl;
  cout<< "LIR for h in y: "<< h_LIR["y"]<< endl;
  cout<< "LIR for h in z: "<< h_LIR["z"]<< endl;


  // ----------- loop over hypotheses --------------
  vector<string> factor_types= return_unique_vector(counters.types);
  for (vector<string>::iterator 
            it_type= factor_types.begin(); 
            it_type != factor_types.end();
            ++it_type) {

    string h_type= *it_type;
    cout<< "----------- Hypothesis "<< h_type <<" ----------"<< "\n\n";

    // DEBUG
    // for (int i= 0; i < counters.A_rows[h_type].size(); ++i) {
    //   cout<< "row:  "<< counters.A_rows[h_type][i].first
    //       << "\ttime: "<< counters.A_rows[h_type][i].second<< endl;
    // }

    // extract the row indexes 
    vector<int> row_inds;
    transform(counters.A_rows[h_type].begin(), 
              counters.A_rows[h_type].end(), 
              back_inserter( row_inds ),
              return_first_element );

    // number of msmts for hypothesis
    int h_n= row_inds.size();

    // DEBUG
    // cout<< "Rows to be extracted: ";
    // printIntVector( row_inds );

    if (h_n == 0) {
      cout<< "empty hypothesis"<< endl;
      continue;
    }
    
    // extract rows & cols from M
    Matrix h_M = extractMatrixRowsAndColumns(M, row_inds, row_inds);
    Eigen::FullPivLU<Matrix> h_M_lu(h_M);
    h_M_lu.setThreshold(1e-8);
    double h_M_rank= h_M_lu.rank();
    cout<< "size of M is = "<< h_M.rows() << " x "<< h_M.cols()<< endl;
    cout << "rank of M is " << h_M_rank << endl; 
    
    // compute LIR only if h_M is full rank
    if (h_M_rank != h_M.rows()){
      cout<< "M for hypothesis is rank deficient"<< endl;
      continue;
    }

    // build vector D
    Eigen::SelfAdjointEigenSolver<Matrix> adj(h_M);
    Matrix h_M_inv_sqrt= adj.operatorInverseSqrt();
    
    // vector D
    map<string, Vector> D;
    D["x"]= h_M_inv_sqrt * extractMatrixRows(S_transpose, row_inds) * t_vector["x"];
    D["y"]= h_M_inv_sqrt * extractMatrixRows(S_transpose, row_inds) * t_vector["y"];
    D["z"]= h_M_inv_sqrt * extractMatrixRows(S_transpose, row_inds) * t_vector["z"];

    // get kappa
    map<string, double> k;
    k["x"]= D["x"].squaredNorm() / var["x"];
    k["y"]= D["y"].squaredNorm() / var["y"];
    k["z"]= D["z"].squaredNorm() / var["z"];

    // non-centrality parameter
    map<string, double> mu;
    mu["x"]= k["x"] * lambda;
    mu["y"]= k["y"] * lambda;
    mu["z"]= k["z"] * lambda;

    // compute integrity
    h_LIR["x"]= 1 - boost::math::cdf(
                        boost::math::non_central_chi_squared(1, mu["x"]),
                        pow(params.AL_x / sqrt(var["x"]), 2) );
    h_LIR["y"]= 1 - boost::math::cdf(
                        boost::math::non_central_chi_squared(1, mu["y"]), 
                        pow(params.AL_y / sqrt(var["y"]), 2) );
    h_LIR["z"]= 1 - boost::math::cdf(
                        boost::math::non_central_chi_squared(1, mu["z"]), 
                        pow(params.AL_z / sqrt(var["z"]), 2) );
    

    cout<< "LIR for h in x: "<< h_LIR["x"]<< endl;
    cout<< "LIR for h in y: "<< h_LIR["y"]<< endl;
    cout<< "LIR for h in z: "<< h_LIR["z"]<< endl;
  
  }



  // // ----------- reduced factor graph --------------
  // {
  //   GaussianFactorGraph fg= lin_graph->clone();
  //   int factor_count= -1;
  //   for (auto factor : fg){
  //     ++factor_count;
  //     if (!factor) {continue;}

  //     string type= counters.types[factor_count];

  //     // remove lidar factors
  //     if (type == "lidar"){fg.remove(factor_count);}
  //     if (type == "marginalized_prior"){fg.remove(factor_count);}
  //   }



  //   Matrix A= fg.jacobian().first;
  //   double n = A.rows(); double m = A.cols();
  //   Eigen::FullPivLU<Matrix> A_lu(A);
  //   A_lu.setThreshold(1e-7);
  //   double A_rank= A_lu.rank();
  //   cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;
  //   cout<< "n = "<< n<< "\nm = "<< m<< endl;
  //   cout<< "rank of A: "<< A_rank<< endl;
  

  //   cout<< "print reduced graph without lidar & marginalized factors"<< endl;
  //   factor_count= -1;
  //   double sum= 0, size= 0;
  //   for (auto factor : fg){
  //     ++factor_count;
  //     if (!factor) {continue;}

  //     // double factor_error= 2 * factor->error(result_fl);
  //     double factor_size= factor->size();
      
  //     cout<< "factor # "<< factor_count<< "\t"
  //         << "type: "<< counters.types[factor_count]<< "\t"
  //         << "size: "<< factor_size<< "\t"
  //         // << "error: "<< 2 * factor_error<< "\t"
  //         << " with keys: ";
  //     factor->printKeys();
  //     // sum += factor_error;
  //     size += factor_size;
  //   }
  //   cout<< "reduce factor size: "<< size<< endl;

  //   cout<< "----------------------"<< endl;
  // }
  // // -----------------------------------


  

  // save factor graph as graphviz dot file
  // Use this to convert to png image
  // dot -Tpng -Gdpi=1000 isam_example.dot -o isam_example.png
  // ofstream os("isam_example.dot");
  // factor_graph.saveGraph(os, result_fl);

  // print path with python
  string command = "python ../python_plot.py";
  system(command.c_str());

  // GTSAM_PRINT(result);

}

