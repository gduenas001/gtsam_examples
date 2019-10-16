
#include "postProcess.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>


void postProcess(Values result,
				  ISAM2Result isam_result,
				  ISAM2 isam,
				  map<string, vector<int>> A_rows_per_type,
          Counters &counters,
          Params &params){

  // get the factor graph & Jacobian from isam
  NonlinearFactorGraph factor_graph = isam.getFactorsUnsafe();
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph = factor_graph.linearize(result);
  Matrix A= (lin_graph->jacobian()).first;
  Matrix Lambda = (lin_graph->hessian()).first;
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

  // get variances
  map<string,double> var;
  Matrix P_x= isam.marginalCovariance(X(counters.current_factor));
  var["x"]= P_x(3,3);
  var["y"]= P_x(4,4);
  var["z"]= P_x(5,5);

  cout<< "variances (x,y,z): "<< "("<< 
          var["x"]<< ", "<< 
          var["y"]<< ", "<< 
          var["z"]<< ")"<< endl;
  Vector t_vector_x= Vector::Zero(m);
  cout<< "number of factors in the graph: "<< counters.current_factor<< endl;
  // t_vector_x(m)= 1;
  
  // upper bound lambda
  double effective_n= getDOFfromGraph(A_rows_per_type);
  double chi_squared_dof= effective_n - m;
  double r= 2 * factor_graph.error(result);
  boost::math::chi_squared_distribution<> chi2_dist(chi_squared_dof);
  double lambda= pow( sqrt(r) + sqrt(boost::math::quantile(chi2_dist, 1-1e-5)), 2 );
  cout<< "effective number of measurements: "<< effective_n<< endl;
  cout<< "DOF of the chi-squared: "<< chi_squared_dof<< endl;
  cout<< "r: "<< r<< endl;
  cout<< "lambda: "<< lambda<< endl;


  // check residuals
  // boost::optional<double> error_after = isam_result.errorAfter;
  // cout<< "error after: "<< error_after.value_or(-1)<< endl;
  // cout<< "error from error() fn: "<< factor_graph.error(result)<< endl;

  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  
  // check matrix M before elimination
  Matrix M = (Eigen::MatrixXd::Identity(n, n) - A*S);
  Eigen::FullPivLU<Matrix> M_lu(M);
  M_lu.setThreshold(1e-7);
  cout<< "size of M = "<< M.rows() << " x "<< M.cols()<< endl;
  cout << "rank of M is " << M_lu.rank() << endl;


  // loop over hypotheses
  for (map<string, vector<int>>::iterator it= A_rows_per_type.begin(); 
       it != A_rows_per_type.end(); 
       ++it){
    string type = it->first;

    cout<< "----------- Hypothesis "<< type <<" ----------"<< "\n\n";
    // if (type == "odom"){continue;}
    vector<int> row_inds= it->second;
    int h_n= row_inds.size();

    cout<< "Rows to be extracted: "<< endl; 
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
      Vector D= h_M_inv_sqrt * extractMatrixRows(S_transpose, row_inds) * t_vector_x;

      // get k and mu
      double k= D.squaredNorm() / var["x"];
      double mu= k * lambda;

      // compute integrity
      boost::math::non_central_chi_squared_distribution<> nc_chi2_dist(1, mu);
      double h_LIR= 1 - boost::math::cdf(nc_chi2_dist, pow(params.AL_x / sqrt(var["x"]), 2) );
      cout<< "LIR for h: "<< h_LIR<< endl;
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


  // // -----------------------------------
  // double sum= 0, dim= 0, whitened_sum= 0;
  // vector<Vector> whitened_errors;
  // for (auto factor : factor_graph){
  //   double factor_error= factor->error(result);
  //   double factor_dim= factor->dim();

  //   // cast nonlinearfactor to noisemodelfactor
  //   boost::shared_ptr<NoiseModelFactor> noise_factor=
  //            boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
  //   Vector whitened_error= noise_factor->whitenedError(result);
  //   whitened_errors.push_back(whitened_error);
  //   whitened_sum += whitened_error.squaredNorm() * 0.5;

  //   cout<< "dim: "<< factor_dim<< "\t"
  //       << "error: "<< factor_error<< endl;
  //   sum += factor_error;
  //   dim += factor_dim;
  // }
  // cout<< "sum of whitened errors: "<< whitened_sum<< endl;
  // cout<< "sum of errors: "<< sum<< endl;
  // cout<< "sum of dimensions: "<< dim<< endl;
  // // -----------------------------------



  // // check whitened error
  // Vector whitened_error = factor_graph.at(0)->whitenedError(result);
  // double error= whitened_error.squaredNorm() * 0.5;
  // cout<< "error in selected factor: "<<
  //         factor_graph.at(0)->error(result)<< 
  //         " or from error function: "<<
  //         error << endl;


  // print path with python
  // string command = "python ../python_plot.py";
  // system(command.c_str());


  // save factor graph as graphviz dot file
  // Use this to convert to png image
  // dot -Tpng -Gdpi=1000 isam_example.dot -o isam_example.png
  // ofstream os("isam_example.dot");
  // factor_graph.saveGraph(os, result);


  // GTSAM_PRINT(result);

}




