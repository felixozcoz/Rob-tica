import math
import numpy as np  
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------

# Returns the tangent points of the outer tangent lines of two circumferences
# rad1: radius of the first circumference
# rad2: radius of the second circumference
# distEjes: distance between the centers of the circumferences
def calculate_tangencial_points(rad1, rad2, distAxes):   
    # coordinates of the centers of the circumferences
    x1, y1 = rad1 ,0    
    x2, y2 = distAxes + rad1 ,0

    # angle between the centers of the circumferences
    gamma = - math.atan2(y2-y1, x2-x1)
    beta = math.asin((rad2-rad1)/distAxes)
    alpha = gamma - beta

    # calculate the tangent points (outer upper line)
    x3 = x1 + rad1 * math.cos( (math.pi/2) - alpha )
    y3 = y1 + rad1 * math.sin( (math.pi/2) - alpha )
    x4 = x2 + rad2 * math.cos( (math.pi/2) - alpha )
    y4 = y2 + rad2 * math.sin( (math.pi/2) - alpha )

    # calculate the tangent points (outer lower line)
    x5 = x1 + rad1 * math.cos( -(math.pi/2) + alpha )
    y5 = y1 + rad1 * math.sin( -(math.pi/2) + alpha )
    x6 = x2 + rad2 * math.cos( -(math.pi/2) + alpha )
    y6 = y2 + rad2 * math.sin( -(math.pi/2) + alpha )

    return (x3, y3), (x4, y4), (x5, y5), (x6, y6)



# Plot the trace of the bicycle circuit test
def plot_trace_bicicleta(rad1, rad2, distEjes, x1, y1, x2, y2):
    # calculate the tangent points
    (x3, y3), (x4, y4), (x5, y5), (x6, y6) = calculate_tangencial_points(rad1, rad2, distEjes)
    
    ang = np.linspace(0, 2*np.pi, 100)
    cx1 = rad1 * np.cos(ang) + x1
    cy1 = rad1 * np.sin(ang) + y1
    cx2 = rad2 * np.cos(ang) + x2
    cy2 = rad2 * np.sin(ang) + y2

    # plot the circumferences 
    plt.plot(cx1, cy1, 'b')
    plt.plot(cx2, cy2, 'g')

    # plot the circumferences centers
    plt.plot(x1, y1, 'bo')
    plt.plot(x2, y2, 'go')

    # plot the tangent points
    plt.plot(x3, y3, 'ro')
    plt.plot(x4, y4, 'ro')
    plt.plot(x5, y5, 'go')
    plt.plot(x6, y6, 'go')
    
    # plot the tangent lines
    plt.plot([x1, x3], [y1, y3], 'r')
    plt.plot([x2, x4], [y2, y4], 'r')
    plt.plot([x3, x4], [y3, y4], 'r')
    plt.plot([x3, x6], [y5, y6], 'r')
    plt.plot([x1, x5], [y1, y5], 'r')
    plt.plot([x2, x6], [y2, y6], 'r')

    plt.axis('equal')
    plt.show()



#plot_trace_bicicleta(20, 40, 100, 0,0, 0, 100)


