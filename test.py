data = [[10, 20, 30], [40, 50, 60], [70, 80, 90]]

all_values = []
for sublist in data:
    all_values.extend(sublist)  # Add all values into a single list

min_val = min(all_values)
max_val = max(all_values)

print(min_val)
print(max_val)

normalized_data = []
for sublist in data:
    normalized_sublist = []
    for value in sublist:
        # Apply the normalization formula
        normalized_value = (value - min_val) / (max_val - min_val)
        normalized_sublist.append(normalized_value)
    normalized_data.append(normalized_sublist)
    
print(normalized_data)