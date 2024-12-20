# import numpy as np
# from datetime import datetime
# from cosner_utils.utils import *
# from cosner_utils.ode_sim import *
# from cosner_utils.plotting_utils import *
# from cosner_utils.pendulum_visualizer import *
# from cosner_utils.utils import getEllipse
# from torchvision.utils import save_image
# from controllers import *
# from ipMobileNetV2 import *
# import torch
# import csv

# os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'


# def printInfo(timestamp):
#     print("Hi!")
#     print("Running the CDC 2022 script with datetime label :", timestamp)


# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     """
#     Set up Data Logging
#     """
#     timestamp = getTimeStamp()
#     printInfo(timestamp)

#     # Run Inverted Pendulum
#     x0 = np.array([[-0.3], [0]])
#     test = InvertedPendulum(x0)
#     ip = PendulumEnv()
#     convert_tensor = transforms.ToTensor()
#     data_output_path = createDataOutputFolder("inverted_pendulum_ics", timestamp)
#     # model_path = "/home/rkcosner/Documents/Research/CDC22/data/trainedModels/bestl2_3_23.pth"
#     model_path = r"C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\1 July 2024\E2E-IL-CBF-main\invertedPendulum\finalModel.pth"
#     e2e_model = MobileNetV2(width_mult=1, states=1, vel_layer_width=200)
#     e2e_model.load_state_dict(torch.load(model_path))
#     e2e_model.eval()


#     convert_tensor = transforms.ToTensor()


#     def e2e_controller(x):
#         theta = x[0, 0]
#         theta_dot = x[1, 0]
#         ip.state = np.array([np.sin(theta), np.cos(theta), theta_dot])
#         ip.last_u = None
#         frame = ip.render(mode="rgb_array")
#         img = Image.fromarray(frame)
#         img = preprocess(img)
#         save_image(img, "current_img.png")

#         img_gen = Image.open("current_img.png")
#         img_gen = convert_tensor(img_gen)

#         theta_dot = torch.tensor([theta_dot], dtype=torch.float32).unsqueeze(0)
#         u = e2e_model(img_gen.unsqueeze(0), theta_dot)
#         u = u.cpu().detach().numpy()
#         return u

#     test.controller = test.trop #e2e_controller #test.trop#
#     test.tend = 1
#     test.dt = 0.01
#     test.simulate = test.simulateEulerStep

#     xs = np.linspace(-1,1,13)
#     ys = np.linspace(-1,1,13)


#     # Get ellipse
#     ex, ey = getEllipse()


#     # data_output_path="C:/Users/saber/OneDrive/Desktop/Research/Pablo-Marek-Begum/Weekly Meeting/1 July 2024/E2E-IL-CBF-main/Data/ic"

#     # iter = 0
#     # for x in xs:
#     #     for y in ys:
#     #         if test.getSafetyVal(np.array([[x],[y]]))>=0:
#     #             # os.system("mkdir " + data_output_path + str(iter))
#     #             x0 = np.array([[x], [y]])
#     #             test.x0 = x0
#     #             test.reset()
#     #             test.simulate()
#     #             test.moduloSpin()
#     #             test.plotPlanar(data_output_path+"/ic" + str(iter), ex, ey)

#     #             test.plotPlanar(data_output_path+"/ic" + str(iter), ex, ey)
#     #             test.saveData(data_output_path+"/ic" + str(iter))

#     #             iter +=1
# import os
# import numpy as np
# import plotly.graph_objs as go
# from plotly.subplots import make_subplots


# # Assuming this is part of your InvertedPendulum class
# class InvertedPendulum:
#     # Existing methods...

#     def plotPlanar(self, save_path, ex, ey, show=True):
#         fig, ax = plt.subplots()
#         # Your existing plotting code here

#         # Save the figure
#         fig.savefig(os.path.join(save_path, 'planar_states.png'))
        
#         # Show the plot only if show is True
#         if show:
#             plt.show()

#         plt.close(fig)  # Close the figure to avoid memory issues


# # Define the base output path
# data_output_path = "C:/Users/saber/OneDrive/Desktop/Research/Pablo-Marek-Begum/Weekly Meeting/1 July 2024/E2E-IL-CBF-main/Data/ic"

# # Ensure the base output path exists
# os.makedirs(data_output_path, exist_ok=True)

# # Function to simulate and collect data
# def simulate_and_collect_data(test, xs, ys):
#     all_traces = []

#     iter = 0
#     for x in xs:
#         for y in ys:
#             if test.getSafetyVal(np.array([[x], [y]])) >= 0:
#                 # Create subdirectory for this iteration
#                 iter_path = os.path.join(data_output_path, "ic" + str(iter))
#                 os.makedirs(iter_path, exist_ok=True)

#                 # Create plots directory inside the iteration directory
#                 plots_path = os.path.join(iter_path, "plots")
#                 os.makedirs(plots_path, exist_ok=True)

#                 # Create csvs directory inside the iteration directory
#                 csvs_path = os.path.join(iter_path, "csvs")
#                 os.makedirs(csvs_path, exist_ok=True)

#                 x0 = np.array([[x], [y]])
#                 test.x0 = x0
#                 test.reset()
#                 test.simulate()
#                 test.moduloSpin()
#                 # Save the plot and data, but do not show the plot
#                 test.plotPlanar(iter_path, ex, ey, show=False)
#                 test.saveData(iter_path)

#                 # Assuming test.simulate() generates a time series of states
#                 states = test.getStates()  # Placeholder for actual function to get states
#                 trace = go.Scatter(
#                     x=states[:, 0].flatten(),
#                     y=states[:, 1].flatten(),
#                     mode='lines',
#                     name=f'Simulation {iter}'
#                 )
#                 all_traces.append(trace)

#                 iter += 1

#     return all_traces

# # Simulate and collect data
# all_traces = simulate_and_collect_data(test, xs, ys)

# # Create interactive Plotly plot
# fig = make_subplots(rows=1, cols=1)

# for trace in all_traces:
#     fig.add_trace(trace)

# fig.update_layout(
#     title="Interactive Simulation Plot",
#     xaxis_title="State Variable 1",
#     yaxis_title="State Variable 2"
# )

# fig.show()




import numpy as np
from datetime import datetime
from cosner_utils.utils import *
from cosner_utils.ode_sim import *
from cosner_utils.plotting_utils import *
from cosner_utils.pendulum_visualizer import *
from cosner_utils.utils import getEllipse
from torchvision.utils import save_image
from controllers import *
from ipMobileNetV2 import *
import torch
import csv

def printInfo(timestamp):
    print("Hi!")
    print("Running the CDC 2022 script with datetime label :", timestamp)


os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    """
    Set up Data Logging
    """
    timestamp = getTimeStamp()
    printInfo(timestamp)

    # Run Inverted Pendulum
    x0 = np.array([[-0.3], [0]])
    test = InvertedPendulum(x0)
    ip = PendulumEnv()
    convert_tensor = transforms.ToTensor()
    data_output_path = createDataOutputFolder("inverted_pendulum_ics", timestamp)
    model_path = r"C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\1 July 2024\E2E-IL-CBF-main\invertedPendulum\finalModel.pth"
    e2e_model = MobileNetV2(width_mult=1, states=1, vel_layer_width=200)
    e2e_model.load_state_dict(torch.load(model_path))
    e2e_model.eval()


    convert_tensor = transforms.ToTensor()


    def e2e_controller(x):
        theta = x[0, 0]
        theta_dot = x[1, 0]
        ip.state = np.array([np.sin(theta), np.cos(theta), theta_dot])
        ip.last_u = None
        frame = ip.render(mode="rgb_array")
        img = Image.fromarray(frame)
        img = preprocess(img)
        save_image(img, "current_img.png")

        img_gen = Image.open("current_img.png")
        img_gen = convert_tensor(img_gen)

        theta_dot = torch.tensor([theta_dot], dtype=torch.float32).unsqueeze(0)
        u = e2e_model(img_gen.unsqueeze(0), theta_dot)
        u = u.cpu().detach().numpy()
        return u, True

    # test.controller = test.trop#e2e_controller #test.trop#
    test.controller = e2e_controller #test.trop#
    test.tend = 1
    test.dt = 0.01
    test.simulate = test.simulateEulerStep

    xs = np.linspace(-1,1,13)
    ys = np.linspace(-1,1,13)


    # Get ellipse
    ex, ey = getEllipse()


    iter = 0
    for x in xs:
        for y in ys:
            if test.getSafetyVal(np.array([[x],[y]]))>=0:
                os.system("mkdir " + data_output_path +"/ic" + str(iter))
                x0 = np.array([[x], [y]])
                test.x0 = x0
                test.reset()
                test.simulate()
                test.moduloSpin()
                test.plotPlanar(data_output_path+"/ic" + str(iter), ex, ey,show=False)
                test.saveData(data_output_path+"/ic" + str(iter))

                iter +=1