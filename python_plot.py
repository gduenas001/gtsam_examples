
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv



# initialize figure
fig = plt.figure()
ax= plt.subplot(111, projection='3d')

# add estimated positions to the plot
estimated_positions_x= []
estimated_positions_y= []
estimated_positions_z= []
with open('estimated_positions.csv', 'rb') as csvfile:
     estimated_positions = csv.reader(csvfile, delimiter=',')
     for position in estimated_positions:
     	estimated_positions_x.append(float(position[0]))
     	estimated_positions_y.append(float(position[1]))
     	estimated_positions_z.append(float(position[2]))


# add estimated positions to the plot
true_positions_x= []
true_positions_y= []
true_positions_z= []
with open('true_positions.csv', 'rb') as csvfile:
     true_positions = csv.reader(csvfile, delimiter=',')
     for position in true_positions:
		true_positions_x.append( float(position[0]) )
		true_positions_y.append( float(position[1]) )
		true_positions_z.append( float(position[2]) )

plt.plot(estimated_positions_x, estimated_positions_y, estimated_positions_z, color= 'b', linestyle='-', marker='o')
# plt.axis('equal')

plt.plot(true_positions_x, true_positions_y, true_positions_z, color= 'r', linestyle='-')
# plt.axis('equal')


max_range = np.array([max(estimated_positions_x) - min(estimated_positions_x), max(estimated_positions_y) - min(estimated_positions_y), max(estimated_positions_z) - min(estimated_positions_z)]).max() / 2.0

mid_x = ( max(estimated_positions_x) + min(estimated_positions_x) ) * 0.5
mid_y = ( max(estimated_positions_y) + min(estimated_positions_y) ) * 0.5
mid_z = ( max(estimated_positions_z) + min(estimated_positions_z) ) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)


# add axis
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


# plt.show(block=False)
# plt.pause(10)
# plt.close()

plt.show()
print "figure closed"













# x = np.linspace(0, 10, 500)
# y = np.sin(x)

# fig, ax = plt.subplots()

# # Using set_dashes() to modify dashing of an existing line
# line1, = ax.plot(x, y, label='Using set_dashes()')
# line1.set_dashes([2, 2, 10, 2])  # 2pt line, 2pt break, 10pt line, 2pt break

# # Using plot(..., dashes=...) to set the dashing when creating a line
# line2, = ax.plot(x, y - 0.2, dashes=[6, 2], label='Using the dashes parameter')

# ax.legend()
# plt.show()