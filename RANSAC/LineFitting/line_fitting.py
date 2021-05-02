import numpy as np 

class LinearLeastSqaureModel:
    def __init__(self, ):
        self.slop=1.0
        self.bias=1.5

    def fit(self, x_arr, y_arr):
        #compute slop
        slop = (len(x_arr) * np.sum(x_arr*y_arr) - np.sum(x_arr) * np.sum(y_arr)) / (len(x_arr)*np.sum(x_arr*x_arr) - np.sum(x_arr) ** 2)
        bias = (np.sum(y_arr) - slop*np.sum(x_arr)) / len(x_arr)
        model = [slop, bias]
        self.slop, self.bias = slop, bias
        return model
        

class RansacModel:
    def __init__(self, curve_fitting_model):
        self.curve_fitting_model = curve_fitting_model

    def fit(self, data, sample_percent=0.8, max_iter=20, threshold=0.01):
        num_iter = 0
        full_x_coords, full_y_coords = data
        num_samples = int(len(full_x_coords)*sample_percent)
        best_slop, best_bias = 1.0, 1.5
        max_num_inliner = 0.0
        while num_iter < max_iter:
            # Shuffle data
            shuffler = np.random.permutation(len(full_x_coords))
            x_coords = full_x_coords[shuffler][:num_samples]
            y_coords = full_y_coords[shuffler][:num_samples]

            # estimate model
            slop, bias = self.curve_fitting_model.fit(x_coords, y_coords)
            y_preds = slop*full_x_coords + bias
            errors = np.abs(full_y_coords - y_preds)
            curr_num_inliner = np.count_nonzero(errors < threshold)
            if curr_num_inliner > max_num_inliner:
                max_num_inliner = curr_num_inliner
                best_slop, best_bias = slop, bias

            num_iter += 1
        self.curve_fitting_model.slop = best_slop
        self.curve_fitting_model.bias = best_bias

def fit_model(data):
    linear_model = LinearLeastSqaureModel()
    ransac_model = RansacModel(linear_model)
    ransac_model.fit(data)
    model = ransac_model.curve_fitting_model
    return model

if __name__ == "__main__":
    # create random point
    x_coords = np.random.uniform(-5.0, 5.0, size=50)
    y_coords = 2*x_coords + 1
    # create outliner
    x_outliners = np.random.uniform(-5.0, 5.0, size=5)
    y_outliners = 2*x_outliners +5
    x_coords = np.concatenate((x_coords, x_outliners), axis=0)
    y_coords = np.concatenate((y_coords, y_outliners), axis=0)
    
    data = [x_coords, y_coords]
    model = fit_model(data)