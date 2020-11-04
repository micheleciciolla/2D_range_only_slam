# Perfoming 2D Slam with a Differential Drive Robot having Range Sensor

![Puppet Forge endorsement](https://img.shields.io/puppetforge/e/camptocamp/openssl?color=light%20green&label=Ubuntu%2016.04%20LTS&logo=Ubuntu)

Project for [Probabilistic Robotics](https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2019-20), Univ. La Sapienza Roma, 2020.

<a href="https://www.dis.uniroma1.it/"><img src="http://www.dis.uniroma1.it/sites/default/files/marchio%20logo%20eng%20jpg.jpg" width="500"></a>


# Dataset description and how data is extracted

* ground_truth.g2o is used to understand how well you're doing wrt where you should be and where landmarks should be.

* initial_guess.g2o is the dataset you use.
Data types:
* VERTEX_XY (landmark known position) = land_id, x, y
* VERTEX_SE2 (current pose) = id, x, y, theta
* EDGE_SE2 (odometry movement) = from_t, to_t, x_t, y_t, theta_t
* EDGE_RANGE_SE2_XY (measurement) = t, land_id, range







<a href=""><img src="https://lh3.googleusercontent.com/proxy/cHWHQt6vNxn2umETcH47LmEiRxl1OH0InjtoYeuRIiwr-_Ndbg3tpLwbnEuqceU82wIDczFyiRdX7LL-4Ywc" width="500"></a>






