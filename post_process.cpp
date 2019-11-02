
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

  // get the factor graph & Jacobian from isam
  // NonlinearFactorGraph factor_graph = isam.getFactorsUnsafe();
  NonlinearFactorGraph factor_graph= fixed_lag_smoother.getFactors();

  // print the keys of the nonlinear factor graph
  factor_graph.keys().print();
  cout<< endl;

  // this one produces the proper hessian but crashes computing the Jacobian
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph_for_hessian= factor_graph.linearize(result_fl);

  cout<< "get A with linearization point in same line"<< endl;
  Matrix A_direct= factor_graph.linearize( 
            fixed_lag_smoother.calculateEstimate())->jacobian().first;
  cout<< "Got it!"<< endl;
  
  // GaussianFactorGraph lin_graph_clone= lin_graph_for_hessian->clone();
  // try casting the GaussianfactorGraph to a jacobian one (if exists)
  // then get the jacobian


  // for(auto factor: *lin_graph_for_hessian) {

  //   cout<< "type of the factor: "<< typeid(factor).name()<< endl;


  //   cout<<"Printing the keys"<< std::endl;
  //   // factor->print();
  //   // factor->printKeys();
  //   cout<< "Accessing the key vector"<< endl;
  //   const KeyVector& key_vector= factor->keys();
  //   cout<< "Size of the key vector: "<< key_vector.size()<< endl;

  // }

  // this has seg fault as allways                  
  JacobianFactor jacobian_factor(*lin_graph_for_hessian);
  cout<< "jacobian factor created"<< endl;
  Matrix A_from_jacobian_factor= jacobian_factor.jacobian().first;
  cout<< "Jacobian matrix from jacobian factor, A size = "<<
       A_from_jacobian_factor.rows()<< " x "<< A_from_jacobian_factor.cols()<< endl;

  // SEGMENTATION FAULT HERE!!
  // I need to create the JacobianFactor some other way, 
  // maybe indicating the keys to include manually...
  // cout<< "try getting the normal Jacobian..."<< endl;
  // std::pair< Matrix, Vector > A_pair(lin_graph_for_hessian->jacobian());
  // Matrix A= A_pair.first;
  // cout<< "normal Jacobian done!"<< endl;


  // JacobianFactor jacobian_combined_factor(*lin_graph_for_hessian);
  // Matrix A_from_hessian= jacobian_combined_factor.jacobian().first;
  // cout<< "Jacobian matrix, A from hessian size = "
  //       << A_from_hessian.rows()<< " x "<< A_from_hessian.cols()<< endl;


  // doesn't crash but computes weird hessian and Jacobian when lag < sim_time
  boost::shared_ptr<HessianFactor>
                  lin_graph= factor_graph.linearizeToHessianFactor(result);

  Matrix A_sparse= lin_graph_for_hessian->sparseJacobian_();
  Matrix hessian= (lin_graph_for_hessian->hessian()).first;
  Matrix Lambda= (lin_graph->information());
  Matrix A= (lin_graph->jacobian()).first;
  Matrix A_TxA= A.transpose() * A;
  Matrix P= Lambda.inverse();
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
  cout<< "From <hessian>: Hessian (Lambda) matrix size = "<<hessian.rows()<< " x "<< hessian.cols()<< endl;
  // cout<< hessian<< endl;
  cout<< "From <information>: Hessian (Lambda) matrix size = "<<Lambda.rows()<< " x "<< Lambda.cols()<< endl;
  // cout<< Lambda<< endl;
  cout<< "From <hessian>: sparse Jacobian matrix size = "<<A_sparse.rows()<< " x "<< A_sparse.cols()<< endl;


  if (hessian.isApprox(Lambda, 0.01)){
     cout<< "hessian and Lambda are equal"<< endl;
  }else{
     cout<< "hessian and Lambda are NOT equal"<< endl;
  }

  if (Lambda.isApprox(A_TxA, 0.01)){
     cout<< "Lambda and A^T*A are equal"<< endl;
  }else{
     cout<< "Lambda and A^T*A are NOT equal"<< endl;
  }


  // get variances for the last state
  // map<string,double> var= getVariancesForLastPose(isam, counters);
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
  double r= 2 * factor_graph.error(result);
  boost::math::chi_squared_distribution<> chi2_dist_lambda(chi_squared_dof);
  double lambda= pow( sqrt(r) + sqrt(boost::math::quantile(chi2_dist_lambda, 1-1e-5)), 2 );
  cout<< "effective number of measurements: "<< effective_n<< endl;
  cout<< "DOF of the chi-squared: "<< chi_squared_dof<< endl;
  cout<< "r: "<< r<< endl;
  cout<< "lambda: "<< lambda<< endl;


  // check residuals
  // boost::optional<double> error_after = isam_result.errorAfter;
  // cout<< "error after: "<< error_after.value_or(-1)<< endl;
  cout<< "error from error() fn: "<< factor_graph.error(result)<< endl;

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

    // cout<< "Rows to be extracted: "; 
    // printIntVector( row_inds );

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
    double factor_error= factor->error(result);
    double factor_dim= factor->dim();

    // cast nonlinearfactor to noisemodelfactor
    boost::shared_ptr<NoiseModelFactor> noise_factor=
             boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
    Vector whitened_error= noise_factor->whitenedError(result);
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
  factor_graph.saveGraph(os, result);


  // GTSAM_PRINT(result);

}

