import numpy as np
import pandas as pd

# Load the .npy file
data = np.load('data.npy')

# Convert the NumPy array to a pandas DataFrame
df = pd.DataFrame(data)

# Define the name for the Excel file
excel_filename = 'output.xlsx'

# Export the DataFrame to Excel
df.to_excel(excel_filename, index=False)

print("Excel file '{}' has been created successfully.".format(excel_filename))
