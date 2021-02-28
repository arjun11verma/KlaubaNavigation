import csv
from matplotlib import pyplot as plt

def plot_histogram(location_values, bin_factor=None):
    if not bin_factor: bin_factor = 0.1
    bin_append = bin_factor
    max_value = max(location_values)
    bins = []

    while (bin_append < max_value):
        bins.append(bin_append)
        bin_append += bin_factor
    
    plt.hist(location_values, bins=bins)
    plt.show()

def generate_data_from_csv(path_to_file):
    real_location = None
    first_line = True
    tdoa_values = []
    location_values = []
    location_dictionary_list = []

    with open(path_to_file) as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')

        for data in data_reader:
            if first_line: 
                first_line = False
            else:
                if not real_location: 
                    real_location = [float(data[-2]), float(data[-1])]
                elif real_location[0] != float(data[-2]) and real_location[1] != float(data[-1]):
                    location_dictionary_list.append({'real_location': real_location, 'tdoa_values': tdoa_values, 'location_values': location_values})
                    real_location = [float(data[-2]), float(data[-1])]
                    tdoa_values = []
                    location_values = []

                tdoa_values.append([float(data[0]), float(data[1])])
                location_values.append([float(data[2]), float(data[3])])
        
        location_dictionary_list.append({'real_location': real_location, 'tdoa_values': tdoa_values, 'location_values': location_values})
    
    return location_dictionary_list

location_data_path = './TagLocationData.csv'
location_dictionary_list = generate_data_from_csv(location_data_path)

for location_dictory in location_dictionary_list:
    location_values = location_dictory['location_values']
    real_location = location_dictory['real_location']

    x_values = [location[0] for location in location_values]
    y_values = [location[1] for location in location_values]

    print(f'Actual X-Location: {real_location[0]}')
    plot_histogram(x_values)



