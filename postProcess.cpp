
#include "postProcess.h"

void postProcess(Values result,
				  ISAM2Result isam_result,
				  ISAM2 isam,
				  map<string, vector<int>> A_rows_per_type){

  // check residuals
  boost::optional<double> error_after = isam_result.errorAfter;
  cout<< "error after: "<< error_after.value_or(-1)<< endl;
  

  // print the error for all the factor 
  NonlinearFactorGraph factor_graph = isam.getFactorsUnsafe();
  boost::shared_ptr<GaussianFactorGraph> 
                  lin_graph = factor_graph.linearize(result);
  Matrix A = (lin_graph->jacobian()).first;
  cout<< "Jacobian matrix, A size = "<< A.rows()<< " x "<< A.cols()<< endl;


  cout<< "----------- Hypothesis 0 ----------"<< "\n\n";
  // number of measurements and states
  double n = A.rows(); double m = A.cols();
  cout<< "n = "<< n<< "\nm = "<< m<< endl;

  // check matrix M before elimination
  Matrix Lambda = (lin_graph->hessian()).first;
  Matrix S = Lambda.inverse() * A.transpose();
  Matrix M = (Eigen::MatrixXd::Identity(n, n) - A*S);
  Eigen::FullPivLU<Matrix> M_lu(M);
  M_lu.setThreshold(1e-5);
  cout<< "size of M = "<< M.rows() << " x "<< M.cols()<< endl;
  cout << "rank of M is " << M_lu.rank() << endl;


  // loop over hypotheses
  for (map<string, vector<int>>::iterator it= A_rows_per_type.begin(); 
       it != A_rows_per_type.end(); 
       ++it){
    string type = it->first;

    cout<< "----------- Hypothesis "<< type <<" ----------"<< "\n\n";

    cout<< "Rows to be extracted: "<< endl; 
    printIntVector( it->second );

    Matrix h_M = extractJacobianRows(M, it->second);
    Eigen::FullPivLU<Matrix> h_M_lu(h_M);
    h_M_lu.setThreshold(1e-5);
    cout<< "size of M is = "<< h_M.rows() << " x "<< h_M.cols()<< endl;
    cout << "rank of M is " << h_M_lu.rank() << endl; 
  }
 
  // Vector error_vector = factor_graph.at(0)->unwhitenedError(result);
  // Vector error_vector = factor->unwhitenedError(result);
  // cout<< "error in selected factor: "<< factor->error(result)<< endl;
  // cout<< error_vector<< endl;
  


  // boost::math::chi_squared_distribution<> chi2_dist(4);
  // cout<< "cdf value: "<< boost::math::quantile(chi2_dist, 1-1e-7)<< endl;

 // // print path with python
  // string command = "python ../python_plot.py";
  // system(command.c_str());


  // save factor graph as graphviz dot file
  // Use this to convert to png image
  // dot -Tpng -Gdpi=1000 isam_example.dot -o isam_example.png
  // ofstream os("isam_example.dot");
  // factor_graph.saveGraph(os, result);


  // GTSAM_PRINT(result);

}




