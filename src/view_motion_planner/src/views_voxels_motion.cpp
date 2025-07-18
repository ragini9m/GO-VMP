#include "view_motion_planner/views_voxels_motion.h"

namespace view_motion_planner
{

/**************************************************************************************************/
/********************************************SubloopLim********************************************/
/**************************************************************************************************/


SubloopLim::SubloopLim(GRBVar** _path_selection, int _num_of_view, int _start_view_id, bool _print_log)
  : path_selection(_path_selection), num_of_view(_num_of_view), start_view_id(_start_view_id), print_log(_print_log)
{
}


std::vector<int> SubloopLim::findPath(int _cur_view, std::vector<bool>& visited, double** x)
{
  std::vector<int> path;
  int cur_view = _cur_view;
  while (true) {
    path.push_back(cur_view);
    visited[cur_view] = true;
    int next_view_id;
    for (next_view_id = 0; next_view_id < num_of_view; next_view_id++) {
      if(visited[next_view_id]) continue;
      if (x[cur_view][next_view_id] > 0.5) {
        break;
      }
    }
    if (next_view_id == num_of_view) {
      break;
    }
    cur_view = next_view_id;
  }
  return path;
}


void SubloopLim::callback()
{
  try {
    if (where == GRB_CB_MIPSOL) {
      // Found an integer feasible solution - does it have one way path without single loop?
      double** x = new double* [num_of_view];
      for (int i = 0; i < num_of_view; i++)
        x[i] = getSolution(path_selection[i], num_of_view);
      std::vector<std::vector<int>> paths;
      std::vector<bool> visited(num_of_view, false);

      // Find the path starting from start_view_id
      paths.push_back(SubloopLim::findPath(start_view_id, visited, x));

      // Find the path starting from other view
      for (int i = 0; i < num_of_view; i++) {
        if (visited[i]) continue;
        bool selected = false;
        for (int j = 0; j< num_of_view; j++)
          if (x[i][j] > 0.5) {
            selected = true;
            break;
          }
        if (selected) paths.push_back(SubloopLim::findPath(i, visited, x));
      }

      if (print_log) {
        std::cout << "Found " << paths.size() << " paths." << std::endl;
        for (int i = 0; i < paths.size(); i++) {
          std::cout << "Path " << i << ": ";
          for (int j = 0; j < paths[i].size() - 1; j++) {
            std::cout << paths[i][j] << "->";
          }
          std::cout << paths[i][paths[i].size() - 1] << std::endl;
        }
      }
      
      // Check for subloops and add constraints to eliminate them
      if (paths.size() > 1) {
        // Add subloop elimination constraint
        for (int i = 1; i < paths.size(); i++) {
          GRBLinExpr subloop = 0;
          for (int j = 0; j < paths[i].size() - 1; j++){
            subloop += path_selection[paths[i][j]][paths[i][j + 1]];
          }
          subloop += path_selection[paths[i][paths[i].size() - 1]][paths[i][0]];
          addLazy(subloop <= paths[i].size() - 1);
        }
      }

      for (int i = 0; i < num_of_view; i++)
        delete[] x[i];
      delete[] x;
      paths.clear();
      paths.shrink_to_fit();
      visited.clear();
      visited.shrink_to_fit();
    }
  }
  catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  }
  catch (...) {
    std::cout << "Error during callback" << std::endl;
  }
}


/**************************************************************************************************/
/*****************************************ViewsVoxelsMotion****************************************/
/**************************************************************************************************/

/*Constructor*/
ViewsVoxelsMotion::ViewsVoxelsMotion()
{
  start_view_id = 0; // To store the index of the start view within the optimizer matrixes
	print_log = false; // To print all the intermediate results
	time_limit = config.max_gurobi_solver_time; // Time limit for optimization in seconds

  // setup linear programming solver
  ROS_INFO_STREAM("Setting up the linear programming solver");

  try
  {
    env = new GRBEnv();
  }
  catch (GRBException& e)
  {
    ROS_INFO_STREAM("Error number: " << e.getErrorCode());
    ROS_INFO_STREAM("Error message: " << e.getMessage());

    // Error due to no internet connection
    if (e.getErrorCode()==10022)
    {
      ROS_ERROR_STREAM("Connect to the internet and run the node again");
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Standard exception caught during initialization: " << e.what());
    // Exit the program cleanly
    std::exit(0);
  } catch (...)
  {
    ROS_ERROR_STREAM("Unknown exception caught during initialization.");
    // Exit the program cleanly
    std::exit(0);
  }
  model = new GRBModel(*env);


  view_selection = nullptr;
  path_selection = nullptr;

  cb = nullptr;
}


void ViewsVoxelsMotion::createVariables()
{
  // Viewpose selection variables - optimizable
  view_selection = new GRBVar[num_of_view];
  // Path selection variables - optimizable
  path_selection = new GRBVar * [num_of_view];
  for (int i = 0; i < num_of_view; i++) {
    view_selection[i] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "view_selection" + std::to_string(i));
    path_selection[i] = new GRBVar[num_of_view];
    for (int j = 0; j < num_of_view; j++) {
      path_selection[i][j] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "path_selection" + std::to_string(i) + std::to_string(j));
    }
  }
  ROS_INFO_STREAM("View selection and path selection variables created");
}

void ViewsVoxelsMotion::setObjective()
{
  // Set objective : smallest motion cost
  for (int i = 0; i < num_of_view; i++) {
    for (int j = 0; j < num_of_view; j++) {
      obj += views_motion[i][j] * path_selection[i][j];
    }
  }
  model->setObjective(obj, GRB_MINIMIZE);
  ROS_INFO_STREAM("Objective function is set");
}


void ViewsVoxelsMotion::viewPathConstraints()
{
  // View and path realtion
  for (int i = 0; i < num_of_view; i++) {
    GRBLinExpr subject_of_view_in;
    GRBLinExpr subject_of_view_out;
    for (int j = 0; j < num_of_view; j++) {
      subject_of_view_in += path_selection[j][i];
      subject_of_view_out += path_selection[i][j];
    }
    model->addConstr(subject_of_view_in <= view_selection[i], "view_path_relation_00inremove" + std::to_string(i));
    model->addConstr(subject_of_view_out <= view_selection[i], "view_path_relation_00outremove" + std::to_string(i));
    model->addConstr(subject_of_view_in + subject_of_view_out >= view_selection[i], "view_path_relation_001remove" + std::to_string(i));

  }
  ROS_INFO_STREAM("View and path relation set");

  // Forbid edge from node back to itself
  for (int i = 0; i < num_of_view; i++)
    path_selection[i][i].set(GRB_DoubleAttr_UB, 0);

  // Forbid start to virtual end
  path_selection[start_view_id][virtual_end_view_id].set(GRB_DoubleAttr_UB, 0);

  // Select start and virtual end
  view_selection[start_view_id].set(GRB_DoubleAttr_LB, 1);
  view_selection[virtual_end_view_id].set(GRB_DoubleAttr_LB, 1);

  // Degree-1 constraints
  for (int i = 0; i < num_of_view; i++) {
    GRBLinExpr subject_of_indgree;
    GRBLinExpr subject_of_outdgree;
    for (int j = 0; j < num_of_view; j++) {
      subject_of_indgree += path_selection[j][i];
      subject_of_outdgree += path_selection[i][j];
    }
    if (i == start_view_id) {
      model->addConstr(subject_of_indgree == 0, "indgree" + std::to_string(i));
      model->addConstr(subject_of_outdgree == 1, "outdgree" + std::to_string(i));
    }
    else if (i == virtual_end_view_id) {
      model->addConstr(subject_of_indgree == 1, "indgree" + std::to_string(i));
      model->addConstr(subject_of_outdgree == 0, "outdgree" + std::to_string(i));
    }
    else {
      model->addConstr(subject_of_indgree <= 1, "indgree" + std::to_string(i));
      model->addConstr(subject_of_outdgree <= 1, "outdgree" + std::to_string(i));
      model->addConstr(subject_of_indgree == subject_of_outdgree, "inoutdgree" + std::to_string(i));
    }
  }
  ROS_INFO_STREAM("Degree-1 constraints set");
}


void ViewsVoxelsMotion::fullCoverageConstraints()
{
  // Full coverage constraints
  for (int j = 0; j < num_of_voxel; j++) {
    GRBLinExpr subject_of_voxel;
    for (int i = 0; i < num_of_view; i++)
      if (connection_graph[i][j] == 1) subject_of_voxel += view_selection[i];
    model->addConstr(subject_of_voxel >= 1, "coverage" + std::to_string(j));
  }
  ROS_INFO_STREAM("Full coverage constraints set");
}


void ViewsVoxelsMotion::explorationConstraints(std::vector<int> view_ids)
{
  // Get the unique indexes of the exploration views to be considered for selecting.
  GRBLinExpr subject_of_exploration;
  // Add each exploration view to the subject_of_exploration
  for (int i = 0; i < view_ids.size(); i++)
    subject_of_exploration += view_selection[view_ids[i]];
  // Set exploration constraint. Atleast config.number_of_exploration_targets number of exploration views should be selected
  // we provide 2 times the number of exploration targets to be selected so that, it can find the least cost path exploration views
  model->addConstr(subject_of_exploration >= std::min(config.number_of_exploration_targets, static_cast<int>(view_ids.size())), "exploration");
  ROS_INFO_STREAM("Exploration constraints set");
}

void ViewsVoxelsMotion::setCallback()
{
  // Set callback function
  cb = new SubloopLim(path_selection, num_of_view, start_view_id, print_log);
	model->setCallback(cb);
  ROS_INFO_STREAM("Callback function is set");
}


void ViewsVoxelsMotion::setModelProperties()
{
  model->set(GRB_IntParam_LazyConstraints, 1);

  // model->set(GRB_IntParam_Threads, 1); 

  // Sets the time Limit for optimization
  model->set("TimeLimit", std::to_string(config.max_gurobi_solver_time));
  ROS_INFO_STREAM("Time limit for optimization is set to " << config.max_gurobi_solver_time << " seconds");

  // Set the termination criteria for the optimization based on the objective value
  model->set("BestObjStop", std::to_string(config.min_obj_value_to_terminate));
}


bool ViewsVoxelsMotion::solvePath()
{
  ROS_INFO_STREAM("Starting to solve the optimization problem");
  double now_time = clock();

  // Optimize model
  try {
    // check if vertex_index is getting updated in the process
    ROS_INFO_STREAM("Number of views(including virtual end node) :" << num_of_view);
    ROS_INFO_STREAM("Size of view selection vector :" << num_of_view);
    ROS_INFO_STREAM("Size of path selection matrix :" << num_of_view << "x" << num_of_view);
    ROS_INFO_STREAM("Number of voxels :" << num_of_voxel);
    ROS_INFO_STREAM("Size of connection graph matrix :" << connection_graph.size() << "x" << connection_graph[0].size());

    model->optimize();
  } catch (GRBException e) {
    ROS_ERROR_STREAM("Error code = " << e.getErrorCode());
    ROS_ERROR_STREAM(e.getMessage());
  } catch (...) {
    ROS_ERROR_STREAM("Exception during optimization");
  }
  
  // If the optimization is not successful, that is solution count is 0, then return false
  if (model->get(GRB_IntAttr_SolCount) == 0) {
    ROS_ERROR_STREAM("No solution found");
    // Find the reason for the failure
    if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      std::cout << "Optimal solution found." << std::endl;
    } else if (model->get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
      std::cout << "Model is infeasible." << std::endl;
      // Compute IIS
      model->computeIIS();
      model->write("model.ilp");
      std::cout << "IIS written to model.ilp" << std::endl;

      // Print information about the IIS
      for (int i = 0; i < model->get(GRB_IntAttr_NumConstrs); ++i) {
        GRBConstr constr = model->getConstr(i);
        if (constr.get(GRB_IntAttr_IISConstr) == 1) {
          std::cout << "Infeasible constraint: " << constr.get(GRB_StringAttr_ConstrName) << std::endl;
        }
      }
    } else if (model->get(GRB_IntAttr_Status) == GRB_UNBOUNDED) {
      std::cout << "Model is unbounded." << std::endl;
    } else {
      std::cout << "Model status: " << model->get(GRB_IntAttr_Status) << std::endl;
    }
    return false;
  }

  if (print_log) {
    // show all connected path
    std::cout << "Selected paths: " << std::endl;
    for (int i = 0; i < num_of_view; i++) {
      for (int j = 0; j < num_of_view; j++) {
        if (path_selection[i][j].get(GRB_DoubleAttr_X) == 1.0) {
          std::cout << i << " " << j << std::endl;
        }
      }
    }
  }
  // show motion cost
  std::cout << "Obj: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
  // show selected views
  std::cout << "Selected views: ";
  for (int i = 0; i < num_of_view; i++)
    if (view_selection[i].get(GRB_DoubleAttr_X) == 1.0) {
      std::cout << i << " ";
      if (i != virtual_end_view_id) selected_view_ids.push_back(i);
    }
  std::cout << std::endl;
  // show path from start view
  std::cout << "Path: ";
  int cur_view = start_view_id;
  while (true) {
    std::cout << cur_view << " ";
    optimal_paths.push_back(cur_view);
    int next_view_id;
    for (next_view_id = 0; next_view_id < num_of_view; next_view_id++)
      if (path_selection[cur_view][next_view_id].get(GRB_DoubleAttr_X) == 1.0) {
        break;
      }
    if (next_view_id == num_of_view) {
      std::cout << "(Virtual End)";
      optimal_paths.pop_back();
      break;
    }
    cur_view = next_view_id;
  }
  std::cout << std::endl;
  std::cout << "Solve Time: " << (clock() - now_time) / CLOCKS_PER_SEC << "s." << std::endl;

  return true;
}

} // namespace view_motion_planner