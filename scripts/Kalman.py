from math import *
from Matrix import Matrix

def gaussian(mean, variance, x):
    return 1/sqrt(2.*pi*variance)*exp(-0.5*(x-mean)**2/variance)


def update(mean1, variance1, mean2, variance2):
    new_mean = 1/(variance1+variance2)*abs(variance2*mean1+variance1*mean2)
    new_variance = 1/(1/variance1+1/variance2)
    return [new_mean, new_variance]


def predict(mean1, variance1, mean2, variance2):
    new_mean = mean1 + mean2
    new_variance = variance1 + variance2
    return [new_mean, new_variance]


def kalman_filter(x, P):
    for n in range(len(measurements)):        
        # measurement update
        Z = Matrix([[measurements[n]]])
        y = Z - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)

        P = (I - (K * H)) * P
        # prediction
        
        x = (F * x) + u
        P = F * P * F.transpose()

    return x,P





mean = 10
variance = 8
mean2 = 13
variance2 = 2

# 1
# print("updated gausian [mean,varianceiance]: ",str(update(mean,variance,mean2,variance2)))
# print("mean` = ",update_mean(mean, variance,variance2, mean2))
# print("variance` = ",update_variance(variance,variance2))

# 2
# for i in range(1000,2000):
#     print("i="+str(i/100.)+" f="+str(gaussian(10.,4.,i/100.)))

# 3
# measurements = [5., 6., 7., 9., 10.]
# motion = [1., 1., 2., 1., 1.]
# measurement_sig = 4.
# motion_sig = 2.
# mu = 0.
# sig = 10000.

# for n in range(len(measurements)):
#     [mu, sig] = update(mu, sig, measurements[n], measurement_sig)
#     print('update: ', [mu, sig])
#     [mu, sig] = predict(mu, sig, motion[n], motion_sig)
#     print('predict: ', [mu, sig])

# 4

measurements = [1, 2, 3]

x = Matrix([[0.], [0.]]) # initial state (location and velocity)
P = Matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = Matrix([[0.], [0.]]) # external motion
F = Matrix([[1., 1.], [0, 1.]]) # next state function
H = Matrix([[1., 0.]]) # measurement function
R = Matrix([[1.]]) # measurement uncertainty
I = Matrix([[1., 0.], [0., 1.]]) # identity matrix

print(kalman_filter(x, P))
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]