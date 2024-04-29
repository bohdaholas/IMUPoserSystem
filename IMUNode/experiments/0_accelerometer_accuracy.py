import numpy as np
import serial

def parse_measurements(input_str):
    parts = input_str.split(' ')
    x = float(parts[1])
    y = float(parts[3])
    z = float(parts[5])
    return x, y, z

def record_measurements():
    N_SAMPLES = 100
    n = 0
    sensor_measurements = []
    ser.flushInput()
    while n < N_SAMPLES:
        line = ser.readline()
        if not line:
            continue
        line_str = line.decode().strip()
        measurement = parse_measurements(line_str)
        print(measurement)
        sensor_measurements.append(measurement)
        n += 1
    return sensor_measurements


scale_factors = np.array([
    [1.020625,   -0.01039672,  0.0607687],
    [0.00573579,  1.02269824, -0.03376039],
    [0.05549599, -0.01260447,  1.01866692]
])

bias = np.array([0.01695379, 0.42904125, 0.0659835])

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    test_n = 0
    sensor_used = "accelerometer"
    # calibration_used = "no_calibration"
    # calibration_used = "bno055_calibration"
    calibration_used = "custom_calibration"

    tests = []
    test_cases = ("+X,0,0", "-X,0,0", "0,+Y,0", "0,-Y,0", "0,0,+Z", "0,0,-Z")
    G_MS2 = 9.81018
    reference_measurements = np.array([
        [G_MS2, 0, 0],
        [-G_MS2, 0, 0],
        [0, G_MS2, 0],
        [0, -G_MS2, 0],
        [0, 0, G_MS2],
        [0, 0, -G_MS2]
    ])
    i = 0
    while i < len(test_cases):
        print(f"Test case: {test_cases[i]}")
        usr_input = input(f"Press enter to record measurements")
        try:
            measurements = record_measurements()
        except:
            print('Error occurred while recording measurements!')
            continue
        np_measurements = np.array(measurements)
        measurement_mean = np.mean(np_measurements, axis=0)
        measurements_errors = np.abs(np_measurements - reference_measurements[i])
        mae = np.mean(measurements_errors, axis=0)
        rmse = np.sqrt(np.mean(np.square(measurements_errors), axis=0))
        test_case_results = (test_cases[i], [measurement_mean, np.mean(mae), np.mean(rmse)])
        print(f"Test case: {test_cases[i]}\nResults: {test_case_results}\n")
        tests.append(test_case_results)
        i += 1

    filename = f'{test_n}_result_{sensor_used}_{calibration_used}.txt'
    with open(filename, 'w') as results_file:
        total_rmse = 0
        total_mae = 0
        for test_case, results in tests:
            measurement_mean, mae, rmse = results
            total_rmse += rmse
            total_mae += mae
            mean_str = ', '.join(format(x, '.5f') for x in measurement_mean)
            result_str = f"Test case: {test_case}\nResults: Mean = [{mean_str}], RMSE = {rmse}, MAE = {mae}\n\n"
            results_file.write(result_str)
        result_str = f"Average MAE for all test cases: {total_mae / len(test_cases)}\n"
        results_file.write(result_str)
        result_str = f"Average RMSE for all test cases: {total_rmse / len(test_cases)}\n"
        results_file.write(result_str)

    print(f"Results have been written to {filename}")
