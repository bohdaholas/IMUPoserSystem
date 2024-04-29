import numpy as np
import serial

def parse_measurements(input_str):
    parts = input_str.split(' ')
    x = float(parts[1])
    y = float(parts[3])
    z = float(parts[5])
    return x, y, z

def record_measurements():
    N_SAMPLES = 1000
    n = 0
    sensor_measurements = []
    ser.flushInput()
    while n < N_SAMPLES:
        line = ser.readline()
        if not line:
            continue
        line_str = line.decode().strip()
        measurement = parse_measurements(line_str)
        print(round(n / N_SAMPLES * 100), measurement)
        sensor_measurements.append(measurement)
        n += 1
    return sensor_measurements

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    test_n = 2
    sensor_used = "magnetometer"
    # calibration_used = "no_calibration"
    calibration_used = "bno055_calibration"
    # calibration_used = "custom_calibration"
    LVIV_MAG_FIELD_STRENGTH = 43.964

    tests = []
    test_cases = ("field strength test",)
    i = 0
    average_strength = None
    while i < len(test_cases):
        print(f"Test case: {test_cases[i]}")
        usr_input = input(f"Press enter to record measurements")
        try:
            measurements = record_measurements()
        except:
            print('Error occurred while recording measurements!')
            continue
        np_measurements = np.array(measurements)
        measurements_norms = np.linalg.norm(np_measurements, axis=1)
        average_field_strength = np.sum(measurements_norms) / len(measurements_norms)
        absolute_errors = np.abs(measurements_norms - LVIV_MAG_FIELD_STRENGTH)
        mae = np.mean(absolute_errors)
        relative_errors = absolute_errors / LVIV_MAG_FIELD_STRENGTH
        mre = np.mean(relative_errors) * 100
        test_case_results = (test_cases[i], [average_field_strength, mae, mre])
        print(f"Test case: {test_cases[i]}\nResults: {test_case_results}\n")
        tests.append(test_case_results)
        i += 1

    filename = f'{test_n}_result_{sensor_used}_{calibration_used}.txt'
    with open(filename, 'w') as results_file:
        for test_case, results in tests:
            average_strength, abs_err, rel_err = results
            result_str = (f"Test case: {test_case}, Reference magnetic field strength: {LVIV_MAG_FIELD_STRENGTH}"
                          f"\nResults: Average magnetic field strength (mT) = {average_strength} Absolute error (mT) = {abs_err}, Relative error (%) = {rel_err}\n\n")
            results_file.write(result_str)

    print(f"Results have been written to {filename}")
