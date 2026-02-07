from quadrotor import QuadrotorModel

quad = QuadrotorModel("./quad/quad.yaml")

quad.ddynamics(0.001).generate("quad_ddyn.c")
