
#include "post_process.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>

#include <typeinfo>

using namespace std;
using namespace gtsam;


void post_process(
          Values result,
          Values result_fl,
				  ISAM2Result isam_result,
				  FixedLagSmoother::Result isam_result_fl,
          ISAM2 &isam,
          IncrementalFixedLagSmoother &fixed_lag_smoother,
          // BatchFixedLagSmoother fixed_lag_smoother,
				  map<string, vector<int>> A_rows_per_type,
          Counters &counters,
          Params &params){


  cout<< "----------------------"<< endl;
  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: 
                                        fixed_lag_smoother.timestamps()) {
    cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
  }
  cout<< "----------------------"<< endl;


  // get the factor graph & Jacobian from isam
  NonlinearFactorGraph factor_graph= fixed_lag_smoother.getFactors();

  // get the linear graph
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph= factor_graph.linearize(result_fl);

  
  // // doesn't crash but computes weird hessian and Jacobian when lag < sim_time
  // boost::shared_ptr<HessianFactor>
  //                 lin_graph= factor_graph.linearizeToHessianFactor(result);

  Matrix hessian= (lin_graph->hessian()).first;
  Matrix A= (lin_graph->jacobian()).first;
  Matrix P= hessian.inverse();
  Matrix S = P * A.transpose();
  Matrix S_transpose= S.transpose();

  // rank of Jacobian A
  Eigen::FullPivLU<Matrix> A_lu(A);
  A_lu.setThreshold(1e-7);
  double A_rank= A_lu.rank();
  cout<< "rank of A: "<< A_rank<< endl;
  
  // number of measurements and states
  double n = A.rows(); double m = A.cols();
  cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;
  cout<< "n = "<< n<< "\nm = "<< m<< endl;
  cout<< "Hessian (Lambda) matrix size = "<<hessian.rows()<< " x "<< hessian.cols()<< endl;
  
  // if (hessian.isApprox(Lambda, 0.01)){
  //    cout<< "hessian and Lambda are equal"<< endl;
  // }else{
  //    cout<< "hessian and Lambda are NOT equal"<< endl;
  // }

  // get variances for the last state
  map<string,double> var= get_variances_for_last_pose(fixed_lag_smoother, counters);
  cout<< "std dev. (roll, pitch, yaw, x, y, z): "<< "("<< 
          sqrt(var["roll"])<< ", "<< 
          sqrt(var["pitch"])<< ", "<< 
          sqrt(var["yaw"])<< ", "<< 
          sqrt(var["x"])<< ", "<< 
          sqrt(var["y"])<< ", "<< 
          sqrt(var["z"])<< ")"<< endl;
  
  // builds a map for vector t for each coordinate TODO: change to lat, long, vert (needs rotations)
  map<string, Vector> t_vector= buildt_vector(m);
  
  // upper bound lambda
  double effective_n= getDOFfromGraph(A_rows_per_type);
  double chi_squared_dof= effective_n - m;
  double r= 2 * factor_graph.error(result_fl);
  boost::math::chi_squared_distribution<> chi2_dist_lambda(chi_squared_dof);
  double lambda= pow( sqrt(r) + sqrt(boost::math::quantile(chi2_dist_lambda, 1-1e-5)), 2 );
  cout<< "effective number of measurements: "<< effective_n<< endl;
  cout<< "DOF of the chi-squared: "<< chi_squared_dof<< endl;
  cout<< "r: "<< r<< endl;
  cout<< "Non-centrality parameter, lambda: "<< lambda<< endl;

  // check residuals
  cout<< "factor graph error: "<< 
          factor_graph.error(result_fl)<< endl;


  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  
  // check matrix M before elimination
  Matrix M = (Eigen::MatrixXd::Identity(n, n) - A*S);
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

  // loop over hypotheses
  for (map<string, vector<int>>::iterator it= A_rows_per_type.begin(); 
       it != A_rows_per_type.end(); 
       ++it){
    string type = it->first;

    cout<< "----------- Hypothesis "<< type <<" ----------"<< "\n\n";
    vector<int> row_inds= it->second;
    int h_n= row_inds.size();

    cout<< "Rows to be extracted: ";
    printIntVector( row_inds );

    Matrix h_M = extractJacobianRows(M, row_inds);
    Eigen::FullPivLU<Matrix> h_M_lu(h_M);
    h_M_lu.setThreshold(1e-7);
    double h_M_rank= h_M_lu.rank();
    cout<< "size of M is = "<< h_M.rows() << " x "<< h_M.cols()<< endl;
    cout << "rank of M is " << h_M_rank << endl; 

    // // SVD of E*M*E
    // Eigen::JacobiSVD<Matrix> svd(h_M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Matrix U= svd.matrixU();
    // Matrix D= svd.singularValues().asDiagonal() ;
    // cout<< "matrix U size = "<< U.rows()<< " x "<< U.cols()<< endl;
    // cout<< "matrix D size = "<< D.rows()<< " x "<< D.cols()<< endl;    
    // Matrix h_M2= U * D * U.transpose();
    // cout<< h_M<< endl;
    // cout<< "Matrix U: \n";
    // cout<< U<< endl;
    // cout<< "Matrix V: \n";
    // cout<< V<< endl;
    // cout<< "Matrix D: \n";
    // cout<< D<< endl;


    if (type != "odom"){
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


    // A*S = A* P * A', rank of E*A
    // Matrix AS= A*S;
    // Matrix h_AS= extractMatrixRows(AS, it->second);
    // h_AS= extractMatrixColumns(h_AS, it->second);
    // Eigen::FullPivLU<Matrix> h_AS_lu(h_AS);
    // h_AS_lu.setThreshold(1e-7);
    // double h_AS_rank= h_AS_lu.rank();
    // cout<< "size of E*A*S*E: "<< h_AS.rows()<< " x "<< h_AS.cols()<< endl;
    // cout<< "rank of E*A*S*E: "<< h_AS_rank<< endl;
    // // cout<< "matrix E*A*S*E: "<< h_AS<< endl;
    // Matrix M2= Matrix::Identity(h_AS.rows(), h_AS.cols()) - h_AS;
    // // cout<< "matrix (I - E*A*S*E): "<< M2<< endl;
    // Eigen::FullPivLU<Matrix> M2_lu(M2);
    // M2_lu.setThreshold(1e-7);
    // double M2_rank= M2_lu.rank();
    // cout<< "size of (I - E*A*S*E): "<< M2.rows()<< " x "<< M2.cols()<< endl;
    // cout<< "rank of (I - E*A*S*E): "<< M2_rank<< endl;


    // if (AS.isApprox(A*P*A.transpose(), 0.01)){
    //   cout<< "AS is equal"<< endl;
    // }else{
    //   cout<< "AS is NOT equal"<< endl;
    // }
  } // end for loop on h-hypotheses


  // -----------------------------------
  double sum= 0, dim= 0, whitened_sum= 0;
  for (auto factor : factor_graph){
    double factor_error= factor->error(result_fl);
    double factor_dim= factor->dim();

    // cast nonlinearfactor to noisemodelfactor
    boost::shared_ptr<NoiseModelFactor> noise_factor=
             boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
    Vector whitened_error= noise_factor->whitenedError(result_fl);
    whitened_sum += whitened_error.squaredNorm() * 0.5;

    factor->printKeys();
    cout<< "dim: "<< factor_dim<< "\t"
        << "error: "<< factor_error<< endl;
    sum += factor_error;
    dim += factor_dim;
  }
  cout<< "sum of whitened errors (*0.5): "<< whitened_sum<< endl;
  cout<< "sum of errors: "<< sum<< endl;
  cout<< "sum of dimensions: "<< dim<< endl;
  // -----------------------------------

  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp : fixed_lag_smoother.timestamps()) {
    cout << "Key: " << key_timestamp.first << 
            "  Time: " << key_timestamp.second << endl;
  }

  // // print path with python
  // string command = "python ../python_plot.py";
  // system(command.c_str());


  // save factor graph as graphviz dot file
  // Use this to convert to png image
  // dot -Tpng -Gdpi=1000 isam_example.dot -o isam_example.png
  ofstream os("isam_example.dot");
  factor_graph.saveGraph(os, result_fl);


  // GTSAM_PRINT(result);

}

