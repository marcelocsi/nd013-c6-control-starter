import pandas as pd
import matplotlib.pyplot as plt

def read_steer_data():
    steer_file = 'steer_pid_data.txt'
    steer_df = pd.read_csv(steer_file, delim_whitespace = True, header = None)
    steer_df.columns = [
        'Iteration',            # 0
        'Error Steering',       # 1
        'Steering Output',      # 2
        'x[k]',                 # 3
        'y[k]',                 # 4
        'x[k-1]',               # 5
        'y[k-1]',               # 6
        'yaw'                   # 7
        ]
    print(f'Steer data:\n{steer_df.head()}\n')
    return steer_df


def read_throttle_data():
    throttle_file = 'throttle_pid_data.txt'
    throttle_df = pd.read_csv(throttle_file, delim_whitespace = True, header = None, usecols = [0, 1, 2, 3])
    throttle_df.columns = ['Iteration', 'Error Throttle', 'Brake Output', 'Throttle Output']
    print(f'Throttle data:\n{throttle_df.head()}\n')
    return throttle_df


def plot_steer_data(steer_df, n_rows):   
    steer_df2 = steer_df[:n_rows]
    steer_df2.plot(x = steer_df.columns[0], y = [steer_df.columns[1], steer_df.columns[2]], kind = 'line')
    steer_df2.plot(x = steer_df.columns[0], y = [
                                                    steer_df.columns[3],
                                                    steer_df.columns[4],
                                                    steer_df.columns[5],
                                                    steer_df.columns[6],
                                                    steer_df.columns[7],
                                                ],
                    kind = 'line')
    plt.show()
 
    
def plot_throttle_data(throttle_df, n_rows):   
    throttle_df2 = throttle_df[:n_rows]
    throttle_df2.plot(x = throttle_df.columns[0], y = [throttle_df.columns[1], throttle_df.columns[2], throttle_df.columns[3]], kind = 'line')
    plt.show()
 
    
def main():
    steer_df = read_steer_data()
    throttle_df = read_throttle_data()
    n_rows = -1 #2000
    plot_steer_data(steer_df, n_rows)
    plot_throttle_data(throttle_df, n_rows)
 
    
if __name__ == '__main__':
    main()
