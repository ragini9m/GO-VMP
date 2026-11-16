/**
 * claude_implementation.cpp
 *
 * GO-VMP ILP (views_voxels_motion, GO-VMP paper Sec. IV-B.2) extended by an
 * overlap-ratio constraint, tested on the random dummy data of int main().
 *
 * Added to the original formulation (a)-(f):
 *   constants   o_ij = |V_i n V_j| / |V_i u V_j|      overlap ratio of views i and j (Jaccard)
 *               a_ij = 1 if o_ij >= tau, else 0       views i and j are registrable
 *   constraint  (g)  sum_{j != i} a_ij * v_j >= k * v_i   for every view i except start and virtual end
 * Meaning: every selected view must overlap at least k other selected views with ratio >= tau.
 * The objective (minimal motion cost) is unchanged. tau <= 0 or k = 0 gives the original GO-VMP model.
 *
 * Everything that differs from views_voxels_motion.cpp is marked with "[overlap]".
 */
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
#include <utility>
#include <ctime>
#include <unordered_map>

#include <octomap/octomap.h> // Assuming Octomap headers are included properly.

#include <gurobi_c++.h> // Assuming Gurobi headers are included properly.

//views_voxels_motion class hpp header
/**
 * @class sublooplim
 * @brief A class derived from GRBCallback for solving a specific optimization problem.
 *
 * This class defines a GRBCallback for finding optimal paths in a given graph.
 * It checks for subloops in the paths found and adds constraints to eliminate them.
 */
class sublooplim : public GRBCallback{
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
	sublooplim(GRBVar** _path_selection, int _num_of_view, int _start_view_id, bool _print_log = false) {
		path_selection = _path_selection;
		num_of_view = _num_of_view;
		start_view_id = _start_view_id;
		print_log = _print_log;
	}

	/**
	 * @brief Find a path starting from the specified view, considering given constraints.
	 * @param _cur_view ID of the current view.
	 * @param visited Vector indicating if a view has been visited or not.
	 * @param x 2D array of solution values.
	 * @return Vector of view IDs representing the path.
	 */
	std::vector<int> find_path(int _cur_view, std::vector<bool>& visited, double** x) {
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

protected:
	/**
	 * @brief Overridden callback function that is executed during optimization.
	 */
	void callback() {
		try {
			if (where == GRB_CB_MIPSOL) {
				// Found an integer feasible solution - does it have one way path without single loop?
				double** x = new double* [num_of_view];
				for (int i = 0; i < num_of_view; i++)
					x[i] = getSolution(path_selection[i], num_of_view);
				std::vector<std::vector<int>> paths;
				std::vector<bool> visited(num_of_view, false);

				// Find the path starting from start_view_id
				paths.push_back(find_path(start_view_id, visited, x));

				// Find the path starting from other view
				for (int i = 0; i < num_of_view; i++) {
					if (visited[i]) continue;
					bool selected = false;
					for (int j = 0; j< num_of_view; j++)
						if (x[i][j] > 0.5) {
							selected = true;
							break;
						}
					if (selected) paths.push_back(find_path(i, visited, x));
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
};

/**
 * @class views_voxels_motion
 * @brief A class for solving an optimization problem related to motion planning with view selection and voxel coverage.
 *
 * This class defines an optimization problem where motion planning is performed considering view selection and voxel coverage.
 */
class views_voxels_motion {
public:
	int start_view_id;                      ///< ID of the starting view.
	int virtual_end_view_id;                ///< ID of the virtual end view.
	int num_of_view;                        ///< Total number of views.
	int num_of_voxel;                       ///< Total number of voxels.
	std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> voxel_id_map;  ///< Mapping of voxel keys to IDs.
	std::vector<std::vector<bool>> connection_graph;  ///< Connection graph indicating voxel coverage by views.
	int time_limit; 					    ///< Time limit for optimization in seconds.
	bool print_log;                         ///< Flag for printing log messages.

	// [overlap] Overlap (registration redundancy) extension
	double overlap_threshold;               ///< tau: views i,j are registrable if o_ij >= tau. <= 0 disables the extension.
	int min_overlap_views;                  ///< k: every selected view needs >= k selected registrable partners.
	std::vector<std::vector<double>> overlap_ratio;  ///< o_ij = |V_i n V_j| / |V_i u V_j| (Jaccard) on the voxel sets.
	std::vector<std::vector<bool>> registrable;      ///< a_ij = 1 if o_ij >= tau.

	// Optimizer components
	GRBEnv* env;                            ///< Gurobi environment.
	GRBModel* model;                        ///< Gurobi optimization model.
	GRBVar* view_selection;                 ///< Decision variables for view selection.
	GRBVar** path_selection;                ///< 2D array of decision variables for path selection.
	GRBLinExpr obj;                         ///< Linear expression for the objective function.
	sublooplim* cb;                         ///< Callback object for subloop elimination.

	// Output
	std::vector<int> selected_view_ids;           ///< List of selected view IDs.
	std::vector<int> optimal_paths;               ///< Optimal path of view IDs.
	bool solved = false;                          ///< [overlap] True if solve() found a solution.
	double objective_value = -1;                  ///< [overlap] Motion cost of the solution.

	/// [overlap] Properties of the registration graph of a solution (nodes: selected views with data, edges: pairs with o_ij >= tau).
	struct registration_report {
		int num_views = 0;        ///< Selected views, start view excluded.
		int num_edges = 0;        ///< Registrable pairs among them.
		int min_degree = 0;       ///< Smallest number of registrable partners of a selected view.
		int num_components = 0;   ///< 1 means all selected views can be registered into one network.
		int num_loops = 0;        ///< Independent loops = edges - views + components. 0 means no redundancy.
		int num_bridges = 0;      ///< Edges whose failure splits the network.
		bool coverage_ok = false; ///< Every voxel is seen by a selected view (constraint e).
	};

	/**
	 * @brief Constructor for the views_voxels_motion class.
	 * @param views_motion Motion cost between views.
	 * @param views_voxels Voxel coverage information for each view.
	 * @param _start_view_id ID of the starting view.
	 * @param _print_log Flag for printing log messages.
	 * @param time_limit Time limit for optimization in seconds.
	 */
	views_voxels_motion(std::vector<std::vector<double>> views_motion, std::vector<std::vector<octomap::OcTreeKey>>& views_voxels, int _start_view_id, int _time_limit = -1, bool _print_log = false, double _overlap_threshold = -1.0, int _min_overlap_views = 0) {
		start_view_id = _start_view_id;
		print_log = _print_log;
		time_limit = _time_limit;
		overlap_threshold = _overlap_threshold;
		min_overlap_views = _min_overlap_views;
		virtual_end_view_id = views_motion.size();
		//check input view number
		if (views_motion.size() != views_voxels.size()) {
			std::cout << "Error: num_of_view: views_motion.size() != views_voxels.size()" << std::endl;
			return;
		}

		//add a virtual end view
		for (int i = 0; i < views_motion.size(); i++)
			views_motion[i].push_back(0);
		std::vector<double> virtual_end_view_motion;
		for (int i = 0; i < views_motion.size() + 1; i++)
			virtual_end_view_motion.push_back(0);
		views_motion.push_back(virtual_end_view_motion);
		num_of_view = views_motion.size();

		if (print_log) {
			//Print Motion Costs Between Viewpoints
			std::cout << "Motion cost: " << std::endl;
			for (int i = 0; i < num_of_view; i++)
				std::cout << "\t" << i;
			std::cout << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				std::cout << i << "\t";
				for (int j = 0; j < num_of_view; j++)
					std::cout << views_motion[i][j] << "\t";
				std::cout << std::endl;
			}
		}

		//create an id map for all view voxels
		num_of_voxel = 0;
		voxel_id_map.clear();
		for (int i = 0; i < views_voxels.size(); i++) {
			for (int j = 0; j < views_voxels[i].size(); j++) {
				if (voxel_id_map.find(views_voxels[i][j]) == voxel_id_map.end()) {
					voxel_id_map[views_voxels[i][j]] = num_of_voxel++;
				}
			}
		}
		if (print_log) std::cout << "num_of_voxel: " << num_of_voxel << std::endl;

		//create a connection graph for all view voxels
		connection_graph.resize(num_of_view);
		for (int i = 0; i < num_of_view; i++) {
			connection_graph[i].resize(num_of_voxel);
			for (int j = 0; j < num_of_voxel; j++) {
				connection_graph[i][j] = 0;
			}
		}
		for (int i = 0; i < views_voxels.size(); i++) {
			for (int j = 0; j < views_voxels[i].size(); j++) {
				connection_graph[i][voxel_id_map[views_voxels[i][j]]] = 1;
			}
		}

		if (print_log) {
			//print connection graph
			std::cout << "connection_graph:" << std::endl;
			std::cout << '\t';
			for (int i = 0; i < num_of_voxel; i++) {
				std::cout << i << '\t';
			}
			std::cout << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				std::cout << i << '\t';
				for (int j = 0; j < num_of_voxel; j++) {
					std::cout << connection_graph[i][j] << '\t';
				}
				std::cout << std::endl;
			}
		}

		//setup linear programming solver
		env = new GRBEnv();
		model = new GRBModel(*env);
		// Must set LazyConstraints parameter when using lazy constraints
		model->set(GRB_IntParam_LazyConstraints, 1);
		// [overlap] Solver log only in verbose mode, so that the comparison table of main() stays readable
		model->set(GRB_IntParam_OutputFlag, print_log ? 1 : 0);

		// Create variables
		view_selection = new GRBVar[num_of_view];
		path_selection = new GRBVar * [num_of_view];
		for (int i = 0; i < num_of_view; i++) {
			view_selection[i] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "view_selection" + std::to_string(i));
			path_selection[i] = new GRBVar[num_of_view];
			for (int j = 0; j < num_of_view; j++) {
				path_selection[i][j] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "path_selection" + std::to_string(i) + std::to_string(j));
			}
		}
		if (print_log) std::cout << "variables created." << std::endl;

		// Set objective : smallest motion cast
		for (int i = 0; i < num_of_view; i++) {
			for (int j = 0; j < num_of_view; j++) {
				obj += views_motion[i][j] * path_selection[i][j];
			}
		}
		model->setObjective(obj, GRB_MINIMIZE);
		if (print_log) std::cout << "objective set." << std::endl;

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
		if (print_log) std::cout << "view and path relation set." << std::endl;

		// Forbid edge from node back to itself
		for (int i = 0; i < num_of_view; i++)
			path_selection[i][i].set(GRB_DoubleAttr_UB, 0);

		// Forbid start to virtual end
		path_selection[start_view_id][virtual_end_view_id].set(GRB_DoubleAttr_UB, 0);
		path_selection[virtual_end_view_id][start_view_id].set(GRB_DoubleAttr_LB, 1);

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
			model->addConstr(subject_of_indgree <= 1, "indgree" + std::to_string(i));
			model->addConstr(subject_of_outdgree <= 1, "outdgree" + std::to_string(i));
			model->addConstr(subject_of_indgree == subject_of_outdgree, "inoutdgree" + std::to_string(i));
		}
		if (print_log) std::cout << "Degree-1 constraints set." << std::endl;

		// Full coverage constraints
		for (int j = 0; j < num_of_voxel; j++) {
			GRBLinExpr subject_of_voxel;
			for (int i = 0; i < num_of_view; i++)
				if (connection_graph[i][j] == 1) subject_of_voxel += view_selection[i];
			model->addConstr(subject_of_voxel >= 1, "coverage" + std::to_string(j));
		}
		if (print_log) std::cout << "Full coverage constraints set." << std::endl;

		// [overlap] Overlap ratio between every pair of views (constants, computed before optimization):
		// o_ij = |V_i n V_j| / |V_i u V_j|, a_ij = 1 if o_ij >= tau
		overlap_ratio.assign(num_of_view, std::vector<double>(num_of_view, 0.0));
		registrable.assign(num_of_view, std::vector<bool>(num_of_view, false));
		for (int i = 0; i < num_of_view; i++) {
			for (int j = i + 1; j < num_of_view; j++) {
				int num_intersection = 0;
				int num_union = 0;
				for (int c = 0; c < num_of_voxel; c++) {
					if (connection_graph[i][c] && connection_graph[j][c]) num_intersection++;
					if (connection_graph[i][c] || connection_graph[j][c]) num_union++;
				}
				double ratio = num_union > 0 ? (double)num_intersection / num_union : 0.0;
				overlap_ratio[i][j] = overlap_ratio[j][i] = ratio;
				registrable[i][j] = registrable[j][i] = (overlap_threshold > 0 && ratio >= overlap_threshold);
			}
		}
		if (print_log) {
			std::cout << "overlap_ratio:" << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				for (int j = 0; j < num_of_view; j++)
					std::cout << overlap_ratio[i][j] << '\t';
				std::cout << std::endl;
			}
		}

		// [overlap] Registration degree constraints (g): sum_{j != i} a_ij * v_j >= k * v_i
		// The start view (no new data) and the virtual end view are excluded.
		if (overlap_threshold > 0 && min_overlap_views > 0) {
			for (int i = 0; i < num_of_view; i++) {
				if (i == start_view_id || i == virtual_end_view_id) continue;
				GRBLinExpr subject_of_overlap;
				for (int j = 0; j < num_of_view; j++) {
					if (j == i || j == start_view_id || j == virtual_end_view_id) continue;
					if (registrable[i][j]) subject_of_overlap += view_selection[j];
				}
				model->addConstr(subject_of_overlap >= min_overlap_views * view_selection[i], "overlap" + std::to_string(i));
			}
			if (print_log) std::cout << "Overlap constraints set." << std::endl;
		}

		// Set callback function
		cb = new sublooplim(path_selection, num_of_view, start_view_id, print_log);
		model->setCallback(cb);

		// Set time limit
		if (time_limit > 0) {
			model->set("TimeLimit", std::to_string(time_limit));
		}

		if (print_log) std::cout<< "Build optimazation model done." << std::endl;
	}

	/**
	 * @brief Destructor for the views_voxels_motion class.
	 */
	~views_voxels_motion() {
		delete env;
		delete model;
		delete[] view_selection;
		for (int i = 0; i < num_of_view; i++)
			delete[] path_selection[i];
		delete[] path_selection;
		delete cb;
		connection_graph.clear();
		connection_graph.shrink_to_fit();
		selected_view_ids.clear();
		selected_view_ids.shrink_to_fit();
		optimal_paths.clear();
		optimal_paths.shrink_to_fit();
	}

	/**
	 * @brief Solve the optimization problem.
	 * @return [overlap] False if no solution exists (the overlap constraints can make the problem infeasible).
	 */
	bool solve() {
		double now_time = clock();
		// Optimize model
		model->optimize();
		// The overlap constraints can make the problem infeasible (e.g. a voxel only visible from a view without a registrable partner).
		if (model->get(GRB_IntAttr_SolCount) == 0) {
			std::cout << "No solution found. Gurobi status: " << model->get(GRB_IntAttr_Status) << " (3 = infeasible)." << std::endl;
			return false;
		}
		if (print_log) {
			//show all connected path
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
			if (next_view_id == num_of_view - 1) {
				std::cout << next_view_id << " ";
				std::cout << "(Virtual End)";
				break;
			}
			cur_view = next_view_id;
		}
		std::cout << std::endl;
		// show overlap ratios between the selected views (registration graph of the solution)
		std::cout << "Overlap ratios between selected views (* = registrable):" << std::endl;
		for (int a = 0; a < selected_view_ids.size(); a++) {
			for (int b = a + 1; b < selected_view_ids.size(); b++) {
				int i = selected_view_ids[a];
				int j = selected_view_ids[b];
				if (i == start_view_id || j == start_view_id) continue;
				std::cout << "  " << i << "-" << j << ": " << overlap_ratio[i][j] << (registrable[i][j] ? " *" : "") << std::endl;
			}
		}
		std::cout << "Solve Time: " << (clock() - now_time) / CLOCKS_PER_SEC << "s." << std::endl;
		objective_value = model->get(GRB_DoubleAttr_ObjVal);
		solved = true;
		return true;
	}

	/**
	 * @brief [overlap] Number of connected components of a graph given by nodes and edges, optionally ignoring one edge.
	 */
	int count_components(const std::vector<int>& nodes, const std::vector<std::pair<int, int>>& edges, int skipped_edge = -1) {
		std::vector<int> component(num_of_view, -1);
		int num_components = 0;
		for (int s = 0; s < nodes.size(); s++) {
			if (component[nodes[s]] != -1) continue;
			std::vector<int> stack(1, nodes[s]);
			component[nodes[s]] = num_components;
			while (!stack.empty()) {
				int cur = stack.back();
				stack.pop_back();
				for (int e = 0; e < edges.size(); e++) {
					if (e == skipped_edge) continue;
					int next = -1;
					if (edges[e].first == cur) next = edges[e].second;
					if (edges[e].second == cur) next = edges[e].first;
					if (next != -1 && component[next] == -1) {
						component[next] = num_components;
						stack.push_back(next);
					}
				}
			}
			num_components++;
		}
		return num_components;
	}

	/**
	 * @brief [overlap] Check the solution independently of the solver and describe its registration graph.
	 * @param tau Overlap threshold used to decide which pairs of selected views are registrable.
	 *            Passed explicitly so that the baseline (extension disabled) can be evaluated at the same threshold.
	 */
	registration_report evaluate(double tau) {
		registration_report report;
		if (!solved) return report;
		// nodes: selected views that carry data (start view and virtual end excluded)
		std::vector<int> nodes;
		for (int i = 0; i < selected_view_ids.size(); i++)
			if (selected_view_ids[i] != start_view_id) nodes.push_back(selected_view_ids[i]);
		// edges: registrable pairs among them
		std::vector<std::pair<int, int>> edges;
		std::vector<int> degree(num_of_view, 0);
		for (int a = 0; a < nodes.size(); a++)
			for (int b = a + 1; b < nodes.size(); b++)
				if (overlap_ratio[nodes[a]][nodes[b]] >= tau) {
					edges.push_back(std::make_pair(nodes[a], nodes[b]));
					degree[nodes[a]]++;
					degree[nodes[b]]++;
				}
		report.num_views = nodes.size();
		report.num_edges = edges.size();
		report.min_degree = nodes.empty() ? 0 : num_of_view;
		for (int a = 0; a < nodes.size(); a++)
			if (degree[nodes[a]] < report.min_degree) report.min_degree = degree[nodes[a]];
		report.num_components = count_components(nodes, edges);
		report.num_loops = report.num_edges - report.num_views + report.num_components;
		for (int e = 0; e < edges.size(); e++)
			if (count_components(nodes, edges, e) > report.num_components) report.num_bridges++;
		// coverage: every voxel must be seen by at least one selected view
		report.coverage_ok = true;
		for (int c = 0; c < num_of_voxel; c++) {
			bool covered = false;
			for (int i = 0; i < selected_view_ids.size(); i++)
				if (connection_graph[selected_view_ids[i]][c]) covered = true;
			if (!covered) report.coverage_ok = false;
		}
		return report;
	}

	/**
	 * @brief Get the set of selected view IDs.
	 * @return Vector containing selected view IDs.
	 */
	std::vector<int> get_views_id_set() {
		return selected_view_ids;
	}
	
	/**
	 * @brief Get the optimal path of view IDs.
	 * @return Vector containing optimal path of view IDs.
	 */
	std::vector<int> get_optimal_path() {
		return optimal_paths;
	}
};

int main()
{
	/**
	 * good test case:
	 * seed 1, view 4, voxel 2, start 1; <1s
	 * seed 1, view 10, voxel 10, start 0; <1s
	 * seed 1, view 10, voxel 100, start 4; <1s
	 * seed 1, view 100, voxel 1000, start 2; <20s
	 * seed 1, view 200, voxel 500, start 3; <50s
	 */

	// Seed the random number generator.
	srand(1); 

	// Set up test parameters.
	int num_of_view = 10;
	int num_of_voxel = 10;
	int start_view_id = 0;
	int time_limit = -1;		// in seconds, -1 for no limit
	bool print_log = false;		// whether to print log

	//Randomly Generating Motion Costs Between Viewpoints
	std::vector<std::vector<double>> views_motion;
	views_motion.resize(num_of_view);
	for (int i = 0; i < num_of_view; i++) {
		views_motion[i].resize(num_of_view);
		for (int j = 0; j < i; j++) {
			views_motion[i][j] = rand() % 100 + 1;
			views_motion[j][i] = views_motion[i][j];
		}
		views_motion[i][i] = 0;
	}

	if (print_log) {
		//Print Motion Costs Between Viewpoints
		std::cout << "Motion cost: " << std::endl;
		for (int i = 0; i < num_of_view; i++)
			std::cout << "\t" << i;
		std::cout << std::endl;
		for (int i = 0; i < num_of_view; i++) {
			std::cout << i << "\t";
			for (int j = 0; j < num_of_view; j++)
				std::cout << views_motion[i][j] << "\t";
			std::cout << std::endl;
		}
	}
	
	
	//Randomly Generate Voxels for Viewpoints
	std::vector<octomap::OcTreeKey> voxels;
	voxels.resize(num_of_voxel);
	for (int i = 0; i < num_of_voxel; i++) {
		voxels[i].k[0] = rand() % 30000;
		voxels[i].k[1] = rand() % 30000;
		voxels[i].k[2] = rand() % 30000;
	}

	//Randomly Allocate Voxels for Viewpoints
	std::vector<std::vector<octomap::OcTreeKey>> views_voxels;
	views_voxels.resize(num_of_view);
	for (int i = 0; i < num_of_view; i++) {
		int sub_num_of_voxel = rand() % num_of_voxel + 1;
		views_voxels[i].resize(sub_num_of_voxel);
		for (int j = 0; j < sub_num_of_voxel; j++)
			views_voxels[i][j] = voxels[rand() % num_of_voxel];
		if(i == start_view_id) views_voxels[i].clear();
	}

	if (print_log) {
		//Print Voxels for Viewpoints
		std::cout << "Views voxels: " << std::endl;
		for (int i = 0; i < num_of_view; i++) {
			std::cout << i << "\t" << "(";
			for (int j = 0; j < views_voxels[i].size(); j++)
				std::cout << views_voxels[i][j].k[0] << "," << views_voxels[i][j].k[1] << "," << views_voxels[i][j].k[2] << ";";
			std::cout << ")" << std::endl;
		}
	}
	
	// [overlap] Solve the same dummy data with the original model and with the overlap constraint (g).
	// tau <= 0 disables the extension, i.e. the first row is the original GO-VMP formulation.
	struct test_case { std::string name; double tau; int k; };
	std::vector<test_case> test_cases;
	test_cases.push_back({ "GO-VMP baseline", -1.0, 0 });
	test_cases.push_back({ "(g) tau=0.3 k=1", 0.3, 1 });
	test_cases.push_back({ "(g) tau=0.3 k=2", 0.3, 2 });
	test_cases.push_back({ "(g) tau=0.4 k=2", 0.4, 2 });
	test_cases.push_back({ "(g) tau=0.5 k=2", 0.5, 2 });
	double baseline_report_tau = 0.3;	// threshold at which the registration graph of the baseline is reported

	std::cout << std::left << std::setw(18) << "model" << std::setw(8) << "cost" << std::setw(16) << "path"
		<< std::setw(7) << "views" << std::setw(7) << "edges" << std::setw(8) << "mindeg" << std::setw(7) << "comps"
		<< std::setw(7) << "loops" << std::setw(9) << "bridges" << "check" << std::endl;

	bool all_checks_passed = true;
	double baseline_cost = -1;
	for (int t = 0; t < test_cases.size(); t++) {
		//Calculate Optimal Coverage Path
		views_voxels_motion* vvm = new views_voxels_motion(views_motion, views_voxels, start_view_id, time_limit, print_log, test_cases[t].tau, test_cases[t].k);
		// silence the per-solve output of solve() unless verbose, the table contains the same information
		std::streambuf* cout_buffer = std::cout.rdbuf();
		if (!print_log) std::cout.rdbuf(nullptr);
		bool feasible = vvm->solve();
		std::cout.rdbuf(cout_buffer);

		std::cout << std::left << std::setw(18) << test_cases[t].name;
		if (!feasible) {
			std::cout << "infeasible" << std::endl;
			delete vvm;
			continue;
		}
		double report_tau = test_cases[t].tau > 0 ? test_cases[t].tau : baseline_report_tau;
		views_voxels_motion::registration_report report = vvm->evaluate(report_tau);
		std::vector<int> optimal_paths = vvm->get_optimal_path();
		std::string path;
		for (int i = 0; i < optimal_paths.size(); i++)
			path += (i ? "-" : "") + std::to_string(optimal_paths[i]);

		// Checks that do not depend on the solver:
		// 1. all voxels covered, 2. every selected view has >= k registrable partners,
		// 3. a constrained model can never be cheaper than the baseline
		bool check = report.coverage_ok;
		if (test_cases[t].tau > 0 && report.min_degree < test_cases[t].k) check = false;
		if (test_cases[t].tau <= 0) baseline_cost = vvm->objective_value;
		else if (baseline_cost >= 0 && vvm->objective_value < baseline_cost - 1e-6) check = false;
		if (!check) all_checks_passed = false;

		std::cout << std::setw(8) << vvm->objective_value << std::setw(16) << path
			<< std::setw(7) << report.num_views << std::setw(7) << report.num_edges << std::setw(8) << report.min_degree
			<< std::setw(7) << report.num_components << std::setw(7) << report.num_loops << std::setw(9) << report.num_bridges
			<< (check ? "ok" : "FAILED") << std::endl;

		// Clean up.
		delete vvm;
	}
	std::cout << "All checks passed: " << (all_checks_passed ? "yes" : "NO") << std::endl;

	return all_checks_passed ? 0 : 1;
}
