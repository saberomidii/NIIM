""" 
This code was used to in our submission to the 2022 Conference on Decision and Control (CDC)

    Paper Title:    "End-to-End Imitation Learning with Safety Guarantees using Control Barrier Functions"
    Authors:        Ryan K. Cosner, Yisong Yue, Aaron D. Ames
    Correspondent:  Ryan K. Cosner (rkcosner@caltech.edu), PhD candidate
    Usage:          Open Access. Free use. If you find this code useful please cite our paper. 

"""

from getpass import getuser
import numpy as np
from datetime import datetime
from cosner_utils.utils import *
from cosner_utils.ode_sim import *
from cosner_utils.plotting_utils import *
from cosner_utils.pendulum_visualizer import *
from torchvision.utils import save_image
from controllers import *
from ipMobileNetV2 import *
import torch
from tqdm import tqdm
import csv


# os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
# save_path_for_plot= r"C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\1 July 2024\E2E-IL-CBF-main\invertedPendulum"


if __name__ == '__main__':
    printInfo()

    # Run Inverted Pendulum
    print("\nWould you like to run the inverted pendulum code?" )
    timestamp = getTimeStamp()

    flag = getUserInfo()
    print(flag)
    if flag:

        # Create Pendulum 
        print("\tGenerating Pendulum")
        x0 = np.array([[-0.3], [0]])
        inverted_pendulum = InvertedPendulum(x0)
        ip = PendulumEnv()
        convert_tensor = transforms.ToTensor()
        # data_output_path = ("inverted_pendulum", timestamp)
        data_output_path = "inverted_pendulum_" + timestamp
        # os.makedirs(data_output_path, exist_ok=True)

        # Load Inverted Pendulum Model
        print("\tLoading Learned Model")
        model_path = r"C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\1 July 2024\E2E-IL-CBF-main\invertedPendulum\finalModel.pth"
        e2e_model = MobileNetV2(width_mult = 1, states = 1, vel_layer_width = 200)
        e2e_model.load_state_dict(torch.load(model_path))
        e2e_model.eval()
        def e2e_controller(x):
            theta = x[0,0]
            theta_dot = x[1,0]
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


        print("\nWould you like to run the inverted pendulum simulation for a single initial condition? ")
        flag = getUserInfo()
        if flag:
            # Run IP Simulation
            controllers = [0, 1, 2]
            for c in controllers: 
                if c == 0: 
                    print("\nWould you like to use the nominal controller? ")
                    flag = getUserInfo()
                    path_suffix = "_nominal"
                    inverted_pendulum.controller = inverted_pendulum.controllerDes #inverted_pendulum.fbLinController#ip.trop#inverted_pendulum.trop#inverted_pendulum.cbfqp #inverted_pendulum.ip_clf # inverted_pendulum.fbLinController
                elif c == 1: 
                    print("\nWould you like to use the TR-OP controller? ")
                    flag = getUserInfo()
                    path_suffix = "_trop"
                    inverted_pendulum.controller = inverted_pendulum.trop
                elif c == 2: 
                    print("\nWould you like to use the end-to-end learned controller? ")
                    flag = getUserInfo()       
                    path_suffix = "_learned"    
                    inverted_pendulum.controller = e2e_controller
       
                if flag: 
                    inverted_pendulum.tend = 2
                    inverted_pendulum.simulate()
                    inverted_pendulum.moduloSpin()
                    inverted_pendulum.plot(data_output_path)
                    inverted_pendulum.plotPlanar(data_output_path)
                    inverted_pendulum.saveData(data_output_path)
                else: 
                    continue


        # Create Inverted Pendulum Data Set
        if True:
            c = np.sqrt(3)*(45*pi/180)**2 # 45 degrees
            x_min = -45/180*pi-0.1765
            x_max = -x_min
            x_now = x_min
            x = []
            y = []

            r = 0.009

            x_last = x_now
            y_last =  (-2*x_now+np.sqrt(4*np.sqrt(3)*(c - np.sqrt(3)*x_now**2) + 4*x_now**2))/2/np.sqrt(3)
            x.append(x_last)
            y.append(y_last)
            rs = []
            i = 0
            while x_now < x_max:
                y_now =  (-2*x_now+np.sqrt(4*np.sqrt(3)*(c - np.sqrt(3)*x_now**2) + 4*x_now**2))/2/np.sqrt(3)

                if ((x_now-x_last)**2 + (y_now-y_last)**2) > r**2:
                    x.append(x_now)
                    y.append(y_now)
                    rs.append(np.sqrt((x_now-x_last)**2 + (y_now-y_last)**2))
                    x_last = x_now
                    y_last = y_now
                x_now += 0.00005
                i+=1

            x = np.array(x)
            x = np.hstack([x,-x])
            y_bottom = -np.array(y)
            y = np.hstack([y,y_bottom])

            plt.figure()
            plt.plot(x,y, '.')
            plt.show()
            # Visualize
            path = "./data/trainingData/training_set_r_" + str(max(rs))
            # os.system("mkdir " + path )
            os.makedirs(path,exist_ok=True)
            for i in range(len(x)):
                theta = x[i]
                ip.state = np.array([np.sin(theta), np.cos(theta), y[i]])
                ip.last_u = None
                frame = ip.render(mode="rgb_array")
                img = Image.fromarray(frame)
                img = preprocess(img)
                save_image(img, path + "/img_"+ str(i).zfill(4) + ".png")

            # os.system("mkdir " + path + "/csv")
            os.makedirs(path + "/csv",exist_ok=True)
            with open(path + "/csv/data.csv", "w", newline="\n") as csvfile:
                fieldnames = ["i", "theta", "theta_dot", "u"]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for i in range(len(x)):
                    theta = x[i]
                    theta_dot = y[i]
                    state = np.array([[theta], [theta_dot]])
                    u = inverted_pendulum.trop(state)
                    # Debugging: Print out the structure of u
                    # print("u:", u)
                    u = u[0]
                    # if isinstance(u, tuple):  # If u is a tuple, convert it to a NumPy array
                    #     u = np.array(u)
                    writer.writerow({"i": str(i).zfill(4), "theta": x[i], "theta_dot": y[i], "u": u[0,0]})

            # compare results
            if True:

                def e2e_controller(x):
                    theta = x[0,0]
                    theta_dot = x[1,0]
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
                    return u


                err = []
                ues = []
                uls = []

                for i in range(len(x)):
                    state = np.array([[x[i]], [y[i]]])
                    ue = inverted_pendulum.trop(state)
                    # print(ue)
                    ue = ue[0]
                    ul = e2e_controller(state)

                    ues.append(ue[0,0])
                    uls.append(ul[0,0])

                plt.figure()
                plt.plot(ues)
                plt.plot(uls)
                plt.show()
        # Test Inverted Pendulum
        if True:


            convert_tensor = transforms.ToTensor()


            def e2e_controller(x):
                theta = x[0]
                theta_dot = x[1]
                print("angle =", theta * 180 / pi)
                ip.state = np.array([np.sin(theta), np.cos(theta), theta_dot])
                ip.last_u = None
                frame = ip.render(mode="rgb_array")
                img = Image.fromarray(frame)
                img = preprocess(img)
                # save_image(img, "current_img.png")
                #
                # img = Image.open("current_img.png")
                # img = convert_tensor(img)
                theta_dot = torch.tensor([theta_dot], dtype=torch.float32)
                u = e2e_model(img.unsqueeze(0), theta_dot)
                u = u.cpu().detach().numpy()
                # print("input = ", u)
                return u,True  

            inverted_pendulum.controller = e2e_controller #inverted_pendulum.trop#
            inverted_pendulum.tend = 5
            inverted_pendulum.dt = 0.01
            inverted_pendulum.simulate = inverted_pendulum.simulateEulerStep
            inverted_pendulum.simulate()
            inverted_pendulum.moduloSpin()
            inverted_pendulum.plot(data_output_path)
            inverted_pendulum.plotPlanar(data_output_path)
            inverted_pendulum.saveData(data_output_path)



print("execuded")
