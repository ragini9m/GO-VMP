#pragma once

#include <gurobi_c++.h>
#include "view_motion_planner/octree_manager.h"
#include "view_motion_planner/vmp_utils.h"
#include <unordered_map>


namespace view_motion_planner
{

/**
 * @class SubloopLim
 * @brief A class derived from GRBCallback for solving a specific optimization problem.
 *
 * This class defines a GRBCallback for finding optimal paths in a given graph.
 * It checks for subloops in the paths found and adds constraints to eliminate them.
 */
class SubloopLim : public GRBCallback{
public:
	GRBVar** path_selection;  ///< 2D array of decision variables representing path selections.
	int num_of_view;          ///< Total number of views.
	int start_view_id;        ///< ID of the starting view.
	bool print_log; 		  ///< Flag for printing log.

	/**
	 * @brief Constructor for sublooplim class.
	 * @param _path_selection 2D array of decision variables representing path selections.
	 * @param _num_of_view Total number of views.
	 * @param _start_view_id ID of the starting view.
	 * @param _print_log Flag for printing log.
	 */
	SubloopLim(GRBVar** _path_selection, int _num_of_view, int _start_view_id, bool _print_log = false);

	/**
	 * @brief Find a path starting from the specified view, considering given constraints.
	 * @param _cur_view ID of the current view.
	 * @param visited Vector indicating if a view has been visited or not.
	 * @param x 2D array of solution values.
	 * @return Vector of view IDs representing the path.
	 */
	std::vector<int> findPath(int _cur_view, std::vector<bool>& visited, double** x);

protected:
	/**
	 * @brief Overridden callback function that is executed during optimization.
	 */
	void callback();
};


/**
 * @class ViewsVoxelsMotion
 * @brief A class for solving an optimization problem related to motion planning with view selection and voxel coverage.
 *
 * This class defines an optimization problem where motion planning is performed considering view selection and voxel coverage.
 */
class ViewsVoxelsMotion {
public:
	int start_view_id;                      ///< ID of the starting view.
	int virtual_end_view_id;                ///< ID of the virtual end view.
	int num_of_view;                        ///< Total number of views.
	int num_of_voxel;                       ///< Total number of voxels.
	std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> voxel_id_map;  ///< Mapping of voxel keys to IDs.
	std::vector<std::vector<bool>> connection_graph;  ///< Connection graph indicating voxel coverage by views. 1 indicates connection and 0 is no connection
	int time_limit; 					    ///< Time limit for optimization in seconds. -1 if no time limit
	bool print_log;                         ///< Flag for printing log messages.
  	std::vector<std::vector<double>> views_motion;  ///< Motion cost between views. 2D matrix of edge value between nodes.
	std::vector<std::vector<octomap::OcTreeKey>> views_voxels;  ///< Voxel coverage information for each view. 2D matrix of variable col size. row is view id and cols are the voxels being viewed at that point

	// Optimizer components
	GRBEnv* env;                            ///< Gurobi environment.
	GRBModel* model;                        ///< Gurobi optimization model.
	GRBVar* view_selection;                 ///< Decision variables for view selection.
	GRBVar** path_selection;                ///< 2D array of decision variables for path selection.
	GRBLinExpr obj;                         ///< Linear expression for the objective function.
	SubloopLim* cb;                         ///< Callback object for subloop elimination.

	// Output
	// slected_view_ids can be a set
	std::vector<int> selected_view_ids;           ///< List of selected view IDs.
	std::vector<int> optimal_paths;               ///< Optimal path of view IDs.

	/**
	 * @brief Constructor for the views_voxels_motion class.
	 * @param views_motion Motion cost between views. 2D matrix of edge value between nodes.
	 * @param views_voxels Voxel coverage information for each view. 2D matrix of variable col size. row is view id and cols are the voxels being viewed at that point
	 * @param _start_view_id ID of the starting view.
	 * @param _print_log Flag for printing log messages.
	 * @param time_limit Time limit for optimization in seconds.
	 */
	ViewsVoxelsMotion();

	/**
	 * @brief To create view selection and path selection variables.
	 */
	void createVariables();

	/**
	 * @brief Set the objective function for optimization.
	 */
	void setObjective();

	/**
	 * @brief Set the constraints related to views and the paths.
	 */
	void viewPathConstraints();

	/**
	 * @brief Set the constraints related to voxel coverage.
	 */
	void fullCoverageConstraints();

	/**
	 * @brief Set the constraints related to exploration if the set_expl_targets_as_constraints flag is set.
	 * @param view_ids List of view IDs to be explored.
	 */
	void explorationConstraints(std::vector<int> view_ids);

	/**
	 * @brief Set the model properties like time limit, objective value cutoff, etc.
	 */
	void setModelProperties();

	/**
	 * @brief Setting callback for subloop elimination.
	 */
	void setCallback();

	/**
	 * @brief Solve the optimization problem.
	 * @return True if the optimization problem is solved successfully.
	 */
	bool solvePath();

	/**
	 * @brief Clears the model for the next iteration of optimization.
	 */
  	void clear()
    {
		// Reset the Gurobi model
		model->reset();

		voxel_id_map.clear();

		connection_graph.clear();
		connection_graph.shrink_to_fit();

		views_motion.clear();
		views_motion.shrink_to_fit();

		views_voxels.clear();
		views_voxels.shrink_to_fit();

        selected_view_ids.clear();
        selected_view_ids.shrink_to_fit();

        optimal_paths.clear();
        optimal_paths.shrink_to_fit();

		// Clear the decision variables
		if (view_selection != nullptr) {
			delete[] view_selection;
			view_selection = nullptr;
		}

		if (path_selection != nullptr)
		{
			for (int i = 0; i < num_of_view; i++)
			{
				delete[] path_selection[i];
			}
			delete[] path_selection;
			path_selection = nullptr;
		}

        // Clear the objective expression
        obj.clear();

        // Reset the callback
		if (path_selection != nullptr)
		{
			delete cb;
        	cb = nullptr;
		}

    }

};

}