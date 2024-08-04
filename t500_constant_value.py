import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Example data extracted from the table
pwm_values = np.array([
  1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136,
  1140, 1144, 1148, 1152, 1156, 1160, 1164, 1168, 1172, 1176,
  1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208, 1212, 1216,
  1220, 1224, 1228, 1232, 1236, 1240, 1244, 1248, 1252, 1256,
  1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288, 1292, 1296,
  1300, 1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336,
  1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368, 1372, 1376,
  1380, 1384, 1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416,
  1420, 1424, 1428, 1432, 1436, 1440, 1444, 1448, 1452, 1456,
  1460, 1464, 1468, 1472, 1476, 1480, 1484, 1488, 1492, 1496,
  1500, 1504, 1508, 1512, 1516, 1520, 1524, 1528, 1532, 1536,
  1540, 1544, 1548, 1552, 1556, 1560, 1564, 1568, 1572, 1576,
  1580, 1584, 1588, 1592, 1596, 1600, 1604, 1608, 1612, 1616,
  1620, 1624, 1628, 1632, 1636, 1640, 1644, 1648, 1652, 1656,
  1660, 1664, 1668, 1672, 1676, 1680, 1684, 1688, 1692, 1696,
  1700, 1704, 1708, 1712, 1716, 1720, 1724, 1728, 1732, 1736,
  1740, 1744, 1748, 1752, 1756, 1760, 1764, 1768, 1772, 1776,
  1780, 1784, 1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816,
  1820, 1824, 1828, 1832, 1836, 1840, 1844, 1848, 1852, 1856,
  1860, 1864, 1868, 1872, 1876, 1880, 1884, 1888, 1892, 1896,
  1900])

thrust_values = np.array([-10.31, -10.21, -10.11, -10.01, -9.91, -9.81, -9.71, -9.61, -9.51, -9.39, 
-9.28, -9.18, -9.07, -8.97, -8.83, -8.70, -8.57, -8.45, -8.33, -8.21, 
-8.08, -7.95, -7.81, -7.67, -7.54, -7.41, -7.29, -7.16, -7.02, -6.89, 
-6.77, -6.66, -6.54, -6.42, -6.29, -6.16, -6.03, -5.90, -5.79, -5.68, 
-5.56, -5.43, -5.30, -5.16, -5.04, -4.92, -4.80, -4.67, -4.55, -4.43, 
-4.30, -4.18, -4.06, -3.95, -3.84, -3.71, -3.60, -3.49, -3.39, -3.29, 
-3.19, -3.08, -2.98, -2.88, -2.79, -2.69, -2.60, -2.50, -2.41, -2.32, 
-2.23, -2.13, -2.04, -1.94, -1.85, -1.75, -1.65, -1.55, -1.46, -1.37, 
-1.28, -1.19, -1.11, -1.02, -0.94, -0.86, -0.78, -0.70, -0.62, -0.55, 
-0.47, -0.39, -0.31, -0.23, -0.16, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
0.00, 0.00, 0.00, 0.00, 0.00, 0.26, 0.37, 0.49, 0.60, 0.71, 0.83, 0.94, 
1.05, 1.17, 1.28, 1.41, 1.53, 1.66, 1.79, 1.92, 2.05, 2.19, 2.32, 2.47, 
2.62, 2.77, 2.90, 3.03, 3.16, 3.30, 3.43, 3.56, 3.70, 3.83, 3.97, 4.12, 
4.26, 4.40, 4.56, 4.71, 4.86, 5.01, 5.17, 5.35, 5.53, 5.70, 5.88, 6.06, 
6.24, 6.43, 6.62, 6.81, 7.00, 7.20, 7.39, 7.58, 7.77, 7.96, 8.15, 8.35, 
8.54, 8.75, 8.95, 9.15, 9.35, 9.55, 9.76, 9.95, 10.15, 10.35, 10.56, 
10.77, 10.98, 11.19, 11.41, 11.62, 11.83, 12.04, 12.24, 12.45, 12.65, 
12.85, 13.06, 13.26, 13.48, 13.70, 13.91, 14.11, 14.31, 14.49, 14.69, 
14.88, 15.07, 15.27, 15.46, 15.66, 15.85, 16.05, 16.24, 16.44])

# # Reshape data for linear regression
# pwm_values = pwm_values.reshape(-1, 1)
# thrust_values = thrust_values.reshape(-1, 1)

# # Linear regression model
# model = LinearRegression().fit(pwm_values, thrust_values)

# # Constants
# K_v = model.coef_[0][0]
# intercept = model.intercept_[0]

# # Plot the data and the linear fit
# plt.scatter(pwm_values, thrust_values, color='blue', label='Data points')
# plt.plot(pwm_values, model.predict(pwm_values), color='red', label=f'Linear fit: K_v={K_v:.2f}, Intercept={intercept:.2f}')
# plt.xlabel('PWM Value (µs)')
# plt.ylabel('Thrust (kgf)')
# plt.legend()
# plt.show()

# print(f"The value of K_v is {K_v:.2f}")
# print(f"The intercept is {intercept:.2f}")



# Fit a 3rd order polynomial
coefficients = np.polyfit(pwm_values.flatten(), thrust_values.flatten(), 3)
poly = np.poly1d(coefficients)

# Evaluate the polynomial at the given PWM values
thrust_predicted = poly(pwm_values)

# Plot the data and the polynomial fit
plt.scatter(pwm_values, thrust_values, color='blue', label='Data points')
plt.plot(pwm_values, thrust_predicted, color='red', label=f'3rd Order Polynomial fit')
plt.xlabel('PWM Value (µs)')
plt.ylabel('Thrust (kgf)')
plt.legend()
plt.show()

# Print the polynomial coefficients
print(f"The coefficients of the 3rd order polynomial are: {coefficients}")

# def convert_velocity_to_pwm(linear_vel, angular_vel, Kv, intercept):
#     # Convert linear velocity to thrust (assuming a direct proportionality for simplicity)
#     thrust_stern = linear_vel  # This might be scaled based on your specific setup
#     # Convert angular velocity to thrust for turning (assuming direct proportionality for simplicity)
#     thrust_bow = angular_vel   # This might be scaled based on your specific setup
    
#     # Calculate PWM for stern thrusters
#     pwm_stern_left = (thrust_stern - intercept) / Kv
#     pwm_stern_right = (thrust_stern - intercept) / Kv
    
#     # Calculate PWM for bow thrusters
#     pwm_bow_left = (thrust_bow - intercept) / Kv
#     pwm_bow_right = (-thrust_bow - intercept) / Kv
    
#     return pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right

# # Example usage
# linear_velocity = 2  # m/s
# angular_velocity = 10  # rad/s
# Kv = 0.03
# intercept = -44.94

# pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right = convert_velocity_to_pwm(linear_velocity, angular_velocity, Kv, intercept)
# print(f"Stern Left PWM: {pwm_stern_left}, Stern Right PWM: {pwm_stern_right}")
# print(f"Bow Left PWM: {pwm_bow_left}, Bow Right PWM: {pwm_bow_right}")


