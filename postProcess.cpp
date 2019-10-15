
#include "postProcess.h"

void postProcess(Values result,
				  ISAM2Result isam_result,
				  ISAM2 isam,
				  map<string, vector<int>> A_rows_per_type){

  // get the factor graph & Jacobian from isam
  NonlinearFactorGraph factor_graph = isam.getFactorsUnsafe();
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph = factor_graph.linearize(result);
  Matrix A= (lin_graph->jacobian()).first;
  Matrix Lambda = (lin_graph->hessian()).first;
  Matrix S = Lambda.inverse() * A.transpose();

  Eigen::FullPivLU<Matrix> A_lu(A);
  A_lu.setThreshold(1e-7);
  double A_rank= A_lu.rank();
  cout<< "rank of A: "<< A_rank<< endl;
  
  // -----------------------------------
  double sum= 0, dim= 0;
  for (auto factor : factor_graph){
    double factor_error= factor->error(result);
    double factor_dim= factor->dim();
    cout<< "dim: "<< factor_dim<< "\t"
        << "error: "<< factor_error<< endl;
    sum += factor_error;
    dim += factor_dim;

    // factor->print();
    // Vector whitened_error= factor->whitened_error(result);
  }
  cout<< "sum of errors: "<< sum<< endl;
  cout<< "sum of dimensions: "<< dim<< endl;
  // -----------------------------------


  // check residuals
  boost::optional<double> error_after = isam_result.errorAfter;
  cout<< "error after: "<< error_after.value_or(-1)<< endl;
  // cout<< "error from error() fn: "<< factor_graph.error(result)<< endl;

  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  // number of measurements and states
  double n = A.rows(); double m = A.cols();
  cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;
  cout<< "n = "<< n<< "\nm = "<< m<< endl;

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

    cout<< "Rows to be extracted: "<< endl; 
    printIntVector( it->second );

    Matrix h_M = extractJacobianRows(M, it->second);
    Eigen::FullPivLU<Matrix> h_M_lu(h_M);
    h_M_lu.setThreshold(1e-7);
    double h_M_rank= h_M_lu.rank();
    cout<< "size of M is = "<< h_M.rows() << " x "<< h_M.cols()<< endl;
    cout << "rank of M is " << h_M_rank << endl; 

    Eigen::JacobiSVD<Matrix> svd(h_M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Matrix U= svd.matrixU();
    Matrix D= svd.singularValues().asDiagonal() ;
    cout<< "matrix U size = "<< U.rows()<< " x "<< U.cols()<< endl;
    cout<< "matrix D size = "<< D.rows()<< " x "<< D.cols()<< endl;    

    Matrix h_M2= U * D * U.transpose();

    // Eigen::SelfAdjointEigenSolver<Matrix> saes(h_M);
    // Matrix h_M_inv_sqrt= saes.operatorInverseSqrt();
    // cout << "The inverse square root of M is: " << endl;
    // cout << saes.operatorInverseSqrt() << endl;
    // cout << "We can also compute it with operatorSqrt() and inverse(). That yields: " << endl;
    // cout << saes.operatorSqrt().inverse() << endl;

    // cout<< h_M<< endl;
    // cout<< "Matrix U: \n";
    // cout<< U<< endl;
    // cout<< "Matrix V: \n";
    // cout<< V<< endl;
    // cout<< "Matrix D: \n";
    // cout<< D<< endl;

    // A*S = A* P * A', rank of E*A
    Matrix h_A= extractMatrixRows(A, it->second);
    Eigen::FullPivLU<Matrix> h_A_lu(h_A);
    h_A_lu.setThreshold(1e-7);
    double h_A_rank= h_A_lu.rank();
    cout<< "rank of E*A: "<< h_A_rank<< endl;


    if (h_M.isApprox(h_M2, 0.01)){
      cout<< "M is equal"<< endl;
    }else{
      cout<< "M is NOT equal"<< endl;
    }
  }


  // // check whitened error
  // Vector whitened_error = factor_graph.at(0)->whitenedError(result);
  // double error= whitened_error.squaredNorm() * 0.5;
  // cout<< "error in selected factor: "<<
  //         factor_graph.at(0)->error(result)<< 
  //         " or from error function: "<<
  //         error << endl;


  // boost::math::chi_squared_distribution<> chi2_dist(4);
  // cout<< "cdf value: "<< boost::math::quantile(chi2_dist, 1-1e-7)<< endl;

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




