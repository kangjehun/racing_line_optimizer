import numpy as np
import random
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import ray

from utils import divide_alphas, merge_alphas, normalize_alphas, reverse_normalize_alphas

class OptimizationResult:
    """
    Stores the optimization result info
    """
    def __init__(self, method, size_of_input):
        self.method = method
        self.success = False
        self.max_iteration_is_reached = False
        self.run_time = 0
        self.iteration = 0
        self.cost = float('inf')
        self.alphas = np.zeros(size_of_input)
        self.update_alphas(self.alphas)
        self.message = ""
    
    def update_alphas(self, alphas):
        self.alphas = alphas
        self.alphas_left, self.alphas_right = divide_alphas(alphas)

class optimizer:
    """
    Do optimization with given algorithm
    """
    def __init__(self, costfunc, getxy, getTrack, x0, method, mutation_bounds, \
                 start, mid1, mid2, end, \
                 desired_cost=1, \
                 delta_straight=0.20, delta_corner=0.20, \
                 max_iteration=500, num_of_population=1000, \
                 convergence_criteria=10, plot_iteration_max = 100,\
                 curvature_compromise_epsilon=1):
        self.costfunc           = costfunc
        self.getxy              = getxy
        self.getTrack           = getTrack
        self.x0                 = x0
        self.method             = method
        self.mutation_bounds    = mutation_bounds
        self.start              = start
        self.mid1               = mid1
        self.mid2               = mid2
        self.end                = end
        self.desired_cost       = desired_cost
        self.delta_straight     = delta_straight
        self.delta_corner       = delta_corner
        self.max_iteration      = max_iteration
        self.num_of_population  = num_of_population
        self.convergence_criteria = convergence_criteria
        self.plot_iteration_max = plot_iteration_max
        self.epsilon            = curvature_compromise_epsilon
        self.num_of_samples     = x0.size
        self.opt_res            = OptimizationResult(method, self.num_of_samples)
        self.optimize()

    def optimize(self):
        # [parallelized computation] remote function of Ray
        if self.method == 'GA_parallel' :
            @ray.remote
            def compute_rankedsolution(i, sol, costfunc):
                eps = sol[i][0]
                alphas = sol[i][1:]
                return (eps * costfunc(alphas), sol[i])
        
        # print("num_of_samples", self.num_of_samples) #[DEBUG]
        costfunc = self.costfunc
        getxy = self.getxy
        getTrack = self.getTrack
        start = self.start
        mid1  = self.mid1
        mid2  = self.mid2
        end   = self.end
        desired_cost = self.desired_cost
        M = self.max_iteration
        N = self.num_of_population
        n = self.num_of_samples
        epsilon = self.epsilon
        delta_straight = self.delta_straight
        delta_corner   = self.delta_corner
        convergence_threshold = self.convergence_criteria
        plot_iteration_max = self.plot_iteration_max
        iterations = []
        costs = []
        last_update_count = convergence_threshold
        mutation_bounds_start     = self.mutation_bounds[0]
        mutation_bounds_segment   = self.mutation_bounds[1]
        # get track info
        track_midline_xy, track_left_boundary, track_right_boundary, \
        track_left_opt_boundary, track_right_opt_boundary = getTrack()
        track_left_boundary = np.array(track_left_boundary)
        track_left_boundary_x = track_left_boundary[:, 0]
        track_left_boundary_y = track_left_boundary[:, 1]
        track_right_boundary = np.array(track_right_boundary)
        track_right_boundary_x = track_right_boundary[:, 0]
        track_right_boundary_y = track_right_boundary[:, 1]
        track_left_opt_boundary = np.array(track_left_opt_boundary)
        track_left_opt_boundary_x = track_left_opt_boundary[:, 0]
        track_left_opt_boundary_y = track_left_opt_boundary[:, 1]
        track_right_opt_boundary = np.array(track_right_opt_boundary)
        track_right_opt_boundary_x = track_right_opt_boundary[:, 0]
        track_right_opt_boundary_y = track_right_opt_boundary[:, 1]

        # [parallelized computation] Initialize Ray
        if self.method == 'PGA' :
            ray.init()

        # variable to store results
        opt_res = self.opt_res
        # generate initial solutions
        sol = np.zeros((N, n + 1))
        for i in range(N):
            sol[i][0] = epsilon
            sol[i][1] = random.uniform(-1, 1) # Large mutation at first point
            for j in range(2, end + 1): # Relatively small mutation at rest points
                if 2 <= j < mid1:
                    sol[i][j] = random.uniform(-delta_straight, delta_straight)
                elif self.mid1 <= j <= self.mid2:
                    sol[i][j] = random.uniform(-delta_corner, delta_corner)
                else:
                    sol[i][j] = random.uniform(-delta_straight, delta_straight)
        for i in range(N):
            for j in range(2, n+1):
                if j == 0 or j == 1 : continue
                else :
                    sol[i][j] = np.clip(sol[i][j-1] + sol[i][j], -1, 1)
        # sol_formatted = np.array2string(sol, precision=3, separator=',', suppress_small=True) #[DEBUG]
        # print(sol_formatted) # [DEBUG]

        ### Initiate Plotting ###
        # Figure 1
        plt.figure(figsize=(8, 4))
        plt.xlabel('Iteration')
        plt.ylabel('Cost')
        plt.title('GA Optimization Real-time Graph')
        # cost plot
        cost_line, = plt.plot([], [], marker='o', markersize=1.5, linestyle='', color='b')
        # add text annotations
        best_cost_text = plt.text(0.80, 0.90, f'Best Cost: {opt_res.cost:.4f}', \
                             transform=plt.gca().transAxes, va='bottom', ha='center', \
                             color='b', fontsize=8)
        current_cost_text = plt.text(0.785, 0.85, f'Current Cost: {opt_res.cost:.4f}', \
                             transform=plt.gca().transAxes, va='bottom', ha='center', \
                             color='r', fontsize=8)
        iteration_text = plt.text(0.83, 0.80, f'Iteration: {opt_res.iteration}', \
                                  transform=plt.gca().transAxes, va='bottom', ha='center', \
                                  color='k', fontsize=8)
        count_text = plt.text(0.87, 0.75, f'Last Update Count: {last_update_count}', \
                                  transform=plt.gca().transAxes, va='bottom', ha='center', \
                                  color='g', fontsize=8)
        # Figure 2
        fig = plt.figure(figsize=(20, 15))
        fig.set_facecolor('white')  # Set the face color of the figure
        fig.suptitle('Optimization Result so far...')  # Set the main title for the entire figure
        subplot = fig.add_subplot(111, aspect='equal')
        # Set labels for x and y axes
        subplot.set_xlabel('local_x')
        subplot.set_ylabel('local_y')
        left_boundary, = subplot.plot([], [], color='black', linewidth=0.5)  # Left boundary of Track
        left_opt_boundary, = subplot.plot([], [], color='gray', linestyle=':', linewidth=1.0)  # Left opt boundary of Track
        right_boundary, = subplot.plot([], [], color='black', linewidth=0.5)  # Right boundary of Track
        right_opt_boundary, = subplot.plot([], [], color='gray', linestyle=':', linewidth=1.0)  # Right opt boundary of Track
        reference_line, = subplot.plot([], [], color='black', linestyle='--', linewidth=0.5)  # Mid Line of Track
        racing_line, = subplot.plot([], [], color='red', linewidth=0.5, label='current racing line')  # Optimized Line
        best_racing_line, = subplot.plot([], [], color='blue', linewidth=0.5, label='best racing line')  # Optimized Line
        plt.legend()
        plt.ion()  # Turn on interactive mode
        plt.tight_layout()  # Adjust subplot layout
        plt.show()
        ########################

        # Start optimization loop
        t0 = time.time()
        for gen in range(M):
            try : 
                rankedsolutions = []
                # termination condition of optimization 1 : reach the desired cost 
                if opt_res.cost < desired_cost:
                    opt_res.success = True
                    opt_res.message = "The desired cost is reached!"
                    break
                # termination condition 2: solution is converged
                if last_update_count == 0:
                    if opt_res.cost < desired_cost:
                        opt_res.success = True
                    opt_res.message = "The solution is converged..."
                    break
                
                # [Parallelized computation]
                if self.method == 'PGA' :
                    sol_id = ray.put(sol)
                    futures = [compute_rankedsolution.remote(i, sol_id, costfunc) \
                               for i in range(N)]
                    rankedsolutions = ray.get(futures)

                ################################################################
                # [Non parallelized computation]
                if self.method == 'GA' :
                    for i in range(N):
                        print("I'm solving solution ", i)
                        eps = sol[i][0]
                        alphas = sol[i][1:]
                        rankedsolutions.append((eps * costfunc(alphas), sol[i]))
                ################################################################

                rankedsolutions.sort(key=lambda x: x[0])
                # Update opt_res only if current cost is lower
                if opt_res.cost > rankedsolutions[0][0]:
                    opt_res.cost = rankedsolutions[0][0]
                    opt_res.update_alphas(rankedsolutions[0][1][1:])
                    last_update_count = convergence_threshold # Reset last_update_count
                else:
                    last_update_count -= 1
                opt_res.iteration += 1

                ### Plotting ###
                # update plot with the last 100 iterations
                iterations.append(opt_res.iteration)
                costs.append(rankedsolutions[0][0])
                # update plot with the last 100 iterations
                # Figure 1 
                plt.figure(1)
                cost_line.set_xdata(iterations)
                cost_line.set_ydata(costs)
                plt.xlim(iterations[0], iterations[-1])
                plt.ylim(min(costs), max(costs))
                # update text annotations
                best_cost_text.set_text(f'Best Cost: {opt_res.cost:.4f}/ Desired Cost: {desired_cost}')
                current_cost_text.set_text(\
                    f'Current Cost: {rankedsolutions[0][0]:.4f}/ Desired Cost: {desired_cost}')
                iteration_text.set_text(f'Iteration: {opt_res.iteration}/ Max Iteration: {M}')
                count_text.set_text(f'Last Update Count: {last_update_count}')
                # Figure 2
                plt.figure(2)
                racingline_xy = getxy(rankedsolutions[0][1][1:])
                best_racingline_xy = getxy(opt_res.alphas)
                left_boundary.set_xdata(track_left_boundary_x)
                left_boundary.set_ydata(track_left_boundary_y)
                left_opt_boundary.set_xdata(track_left_opt_boundary_x)
                left_opt_boundary.set_ydata(track_left_opt_boundary_y)
                right_boundary.set_xdata(track_right_boundary_x)
                right_boundary.set_ydata(track_right_boundary_y)
                right_opt_boundary.set_xdata(track_right_opt_boundary_x)
                right_opt_boundary.set_ydata(track_right_opt_boundary_y)
                reference_line.set_xdata(track_midline_xy[0])
                reference_line.set_ydata(track_midline_xy[1])
                racing_line.set_xdata(racingline_xy[0])
                racing_line.set_ydata(racingline_xy[1])
                best_racing_line.set_xdata(best_racingline_xy[0])
                best_racing_line.set_ydata(best_racingline_xy[1])
                subplot.set_xlim(min(track_midline_xy[0]) - 20, max(track_midline_xy[0]) + 20)
                subplot.set_ylim(min(track_midline_xy[1]) - 20, max(track_midline_xy[1]) + 20)
                plt.pause(0.01)  # Pause to update the plot
                ###############

                # limit the data to the last plot_iteration_max iterations
                if len(iterations) > plot_iteration_max:
                    iterations.pop(0)
                    costs.pop(0)
                # select survived individuals
                bestsolutions = rankedsolutions[:N // 10]
                # print(bestsolutions) #[DEBUG]
                # normalize the solutions
                norm_eps_arr, norm_s_arr, norm_s1_arr, norm_c_arr, norm_s2_arr \
                    = [], [], [], [], []
                for best_sol in bestsolutions:
                    eps = best_sol[1][0]
                    s = best_sol[1][1]
                    s1 = best_sol[1][2:mid1] - best_sol[1][1:mid1-1]
                    c = best_sol[1][mid1:mid2+1] - best_sol[1][mid1-1:mid2]
                    s2 = best_sol[1][mid2+1:self.end+1] - best_sol[1][mid2:self.end]
                    norm_eps_arr.append(eps) # no need to normalize
                    norm_s_arr.append(normalize_alphas(s))
                    norm_s1_arr.append(normalize_alphas(s1))
                    norm_c_arr.append(normalize_alphas(c))
                    norm_s2_arr.append(normalize_alphas(s2))
                # print("eps : ", norm_eps_arr) # [DEBUG]
                # print("s : ", norm_s_arr) # [DEBUG]
                # produce new generation by using crossover and mutation
                new_sol = []
                for _ in range(N):
                    eps_new = reverse_normalize_alphas(\
                        random.choice(norm_eps_arr)) # No mutation on eps
                    # print("eps_new : ", eps_new) # [DEBUG]
                    s_new = np.clip(reverse_normalize_alphas(\
                                    random.choice(norm_s_arr) * \
                                    random.uniform(mutation_bounds_start[0],
                                                   mutation_bounds_start[1])), \
                                                   -1, 1) 
                                    # starting point is relatively free from mutation
                    s1_new = np.clip(reverse_normalize_alphas(\
                                    random.choice(norm_s1_arr) * \
                                    random.uniform(mutation_bounds_segment[0], \
                                                   mutation_bounds_segment[1])), \
                                                   -delta_straight, delta_straight) 
                                    # 6% mutation for straight segment
                    # print("s1_new : ", s1_new) # [DEBUG]
                    # print("mutation_bounds_start : ", \
                    #       mutation_bounds_start[0], mutation_bounds_start[1])
                    c_new = np.clip(reverse_normalize_alphas(\
                                    random.choice(norm_c_arr) * \
                                    random.uniform(mutation_bounds_segment[0], \
                                                   mutation_bounds_segment[1])), \
                                                   -delta_corner, delta_corner) 
                                    # 6% mutation for corner segment
                    s2_new = np.clip(reverse_normalize_alphas(\
                                     random.choice(norm_s2_arr) * \
                                     random.uniform(mutation_bounds_segment[0], \
                                                    mutation_bounds_segment[1])), \
                                                    -delta_straight, delta_straight) 
                                    # 6% mutation
                    new_sol_element = np.concatenate(([eps_new], [s_new], s1_new, c_new, s2_new))
                    new_sol.append(new_sol_element)
                new_sol = np.array(new_sol)
                # print(new_sol)
                for i in range(N):
                    for j in range(2, n+1):
                        if j == 0 or j == 1 : continue
                        else :
                            new_sol[i][j] = np.clip(new_sol[i][j-1] + new_sol[i][j], -1, 1)
                # print(new_sol)
                sol = new_sol
                # print("iteration            :", opt_res.iteration) # [DEBUG]
                # print("current best cost    :", opt_res.cost) # [DEBUG]
            except KeyboardInterrupt:
                opt_res.message = "The optimization is aborted by command line input (Ctrl+C)"
                if len(rankedsolutions) != 0 :
                    rankedsolutions.sort(key=lambda x: x[0])
                    # Update opt_res only if current cost is lower
                    if opt_res.cost > rankedsolutions[0][0]:
                        opt_res.cost = rankedsolutions[0][0]
                        opt_res.update_alphas(rankedsolutions[0][1][1:])
                        costfunc(opt_res.alphas) # to update alphas & profile
                    else :
                        costfunc(opt_res.alphas) # to update alphas & profile
                    break
        # save opt result
        if opt_res.iteration < self.max_iteration:
            opt_res.max_iteration_is_reached = False
        else:
            opt_res.max_iteration_is_reached = True
        opt_res.run_time = time.time() - t0
        # Turn off interactive mode after the loop completes
        plt.ioff()
        plt.show(block=False)  # Display the final plot
        # [parallelized computation] Shutdown Ray
        if self.method == 'PGA' :
            ray.shutdown()
        return opt_res

        


