Header header

# Inclinometer data
uint16[] image
float32 tempCelcius

# Sun Vector 
float32[] sunVector

# Error code 1 from the sun vector computation algorithm
# The value of the error code should be interpreted as follows:
# >= 0: Successful image.
#   -1: Error returns during NLSQ fit, solution may be valid to nearest pixel.
#   -2: Too many peaks, solution not assigned.
#   -3: Too few peaks, solution not assigned 
#   -4: Appropriate number of peaks found, but fit is of poor quality.
int32 errorCode1

# Error code 2 from the sun vector computation algorithm
# The value of the error code should be interpreted as follows:
#    0: Good geometry.
#   -5: Imaginary solution.
int32 errorCode2