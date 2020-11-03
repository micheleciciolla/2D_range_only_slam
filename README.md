# Perfoming 2D Slam with a Differential Drive Robot having Range Sensor

![Puppet Forge endorsement](https://img.shields.io/puppetforge/e/camptocamp/openssl?color=light%20green&label=Ubuntu%2016.04%20LTS&logo=Ubuntu)

Project for [Probabilistic Robotics](https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2019-20), Univ. La Sapienza Roma, 2020.

<a href="https://www.dis.uniroma1.it/"><img src="http://www.dis.uniroma1.it/sites/default/files/marchio%20logo%20eng%20jpg.jpg" width="500"></a>


# Dataset description and how data is extracted
* initial_guess.g2o
* ground_truth.g2o

Data types:
* VERTEX_XY (landmark known position) = land_id, x, y
* VERTEX_SE2 (current pose) = id, x, y, theta
* EDGE_SE2 (movement) = from_t, to_t, x_t, y_t, theta_t
* EDGE_RANGE_SE2_XY (measurement) = t, land_id, range





