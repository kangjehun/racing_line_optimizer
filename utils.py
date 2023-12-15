import numpy as np
import sys

def is_closed(left, right):
  """
  Compares the first and last cones in each boundary to determine if a track is
  open ended or a closed loop.
  """
  return all(left[:,0]==left[:,-1]) and all(right[:,0]==right[:,-1])

def is_closed(mid):
  """
  Compares the first and last coordinate in each boundary to determine if a track is
  open ended or a closed loop.
  """
  return all(mid[:,0]==mid[:,-1])

def divide_alphas(alphas):
    """divide alphas [-1, 1] to alphas_left and alphas_right [0, 1]"""
    alphas_left = np.zeros_like(alphas)
    alphas_right = np.zeros_like(alphas)
    for i, alpha in enumerate(alphas):
        if -1 <= alpha <= 0:
            alphas_left[i]  = abs(alpha)
            alphas_right[i] = 0
        elif 0 < alpha <= 1:
            alphas_left[i]  = 0
            alphas_right[i] = alpha
        else:
            raise ValueError("Elements in alphas must be within the range [-1, 1]")
    return alphas_left, alphas_right


def merge_alphas(alphas_left, alphas_right):
    if not all(0 <= alpha_left <= 1 for alpha_left in alphas_left) \
       or not all(0 <= alpha_right <= 1 for alpha_right in alphas_right):
        raise ValueError("All elements in alphas_left and alphas_right must be within the range [0, 1]")
    if alphas_left.shape != alphas_right.shape:
        raise ValueError("alphas_left and alphas_right must have the same size")
    for alpha_left, alpha_right in zip(alphas_left, alphas_right):
        if alpha_left != 0 and alpha_right != 0:
            raise ValueError("Both alphas_left and alphas_right cannot have non-zero values for the same index")
    alphas = alphas_left * -1 + alphas_right
    return alphas

def normalize_alphas(alphas, alpha_min=-1, alpha_max=1):
    return (alphas-alpha_min)/(alpha_max-alpha_min)

def reverse_normalize_alphas(norm_alphas, alpha_min=-1, alpha_max=1):
    return (norm_alphas * (alpha_max - alpha_min)) + alpha_min

def xy2xy(v_arr, src_xy_arr, src_is_closed, dest_xy_arr, dest_is_closed):
    """ Convert v_arr from src_xy_arr size to dest_xy_arr size"""
    src_size = len(src_xy_arr[0]) - int(src_is_closed)
    dest_size = len(dest_xy_arr[0]) - int(dest_is_closed)
    # print(v_arr) # [DEBUG]
    # print("track :", src_xy_arr) #[DEBUG]
    # print("spline:", dest_xy_arr) #[DEBUG]
    # v_arr_size = len(v_arr) - int(src_is_closed) #[DEBUG]
    # print("v_arr_size:", v_arr.size) #[DEBUG]
    # print("src size:", src_size) #[DEBUG]
    # print("dest_size:", dest_size) #[DEBUG]
    # print("src_is_closed:", src_is_closed) #[DEBUG]
    v_arr_converted = np.empty(dest_size)  

    min_distance = float('inf')
    distance = float('inf')
    closest_src_point = None
    for j in range(src_size):
        distance = np.linalg.norm(dest_xy_arr[:,0] - src_xy_arr[:,j])
        if distance < min_distance:
            min_distance = distance
            closest_src_point = j
            # print(closest_src_point) #[DEBUG]
    v_arr_converted[0] = v_arr[closest_src_point]
    for i in range(1, dest_size):
        min_distance = float('inf')
        is_updated = False
        # print(">i:", i) #[DEBUG]
        for j in range(src_size if src_is_closed else (src_size - closest_src_point)):
            idx_current = (closest_src_point + j) % src_size
            # print("closed loop count : ", src_size) #[DEBUG]
            # print("j: ", j) #[DEBUG]
            # print("idx_current: ", idx_current) #[DEBUG]
            # print("dest_size : ", dest_size) #[DEBUG]
            # print("open loop count", src_size - closest_src_point) #[DEBUG]
            distance = np.linalg.norm(dest_xy_arr[:, i] - src_xy_arr[:, idx_current])
            if distance < min_distance:
                min_distance = distance
                closest_src_point = idx_current
            else:
                # print("closest point: ", closest_src_point) #[DEBUG]
                v_arr_converted[i] = v_arr[closest_src_point]
                is_updated = True
                break
        if not is_updated : v_arr_converted[i] = v_arr[closest_src_point]
    return v_arr_converted

def remove_duplicate_points(mid_xy_arr, arrays):
    """ Remove duplicate points in track data other than closed track. """
    # check if the track is closed or open
    closed = is_closed(mid_xy_arr)
    # Create a mask of unique indices
    _, unique_indices = np.unique(mid_xy_arr, axis=1, return_index=True)
    # np.set_printoptions(threshold=np.inf) #[DEBUG]
    # print(unique_indices) #[DEBUG]
    # np.set_printoptions(threshold=False) #[DEBUG]
    mask = np.zeros(mid_xy_arr.shape[1], dtype=bool)
    mask[unique_indices] = True
    # Filter the track to remove duplicate points
    filtered_mid_xy_arr = mid_xy_arr[:, mask]
    filtered_arrays = [arr[mask] for arr in arrays]
    if closed :
        filtered_mid_xy_arr = \
            np.hstack((filtered_mid_xy_arr, filtered_mid_xy_arr[:, 0][:, np.newaxis]))
        for i in range(len(filtered_arrays)):
            filtered_arrays[i] = np.append(filtered_arrays[i], filtered_arrays[i][0])
    return filtered_mid_xy_arr, filtered_arrays

def match_dimensions(np_variable):
    """ Match dimenstions of np variable as 2d np array. """
    if np_variable.ndim == 1:
        if np_variable.size == 0:
            np_variable = np.array([[]])
        else:
            np_variable = np_variable.reshape(1, -1)
    return np_variable