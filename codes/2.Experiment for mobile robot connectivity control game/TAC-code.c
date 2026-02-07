std::vector<double> get_dydt(double t, const std::vector<double>& y) {
        std::vector<double> dydt(N * (mi + N * mi), 0.0);

        std::vector<double> pf(N * mi, 0.0);
        std::vector<std::vector<double>> u(N, std::vector<double>(mi, 0.0));

        double delta = 0.01;
        double alpha = 1;

        std::vector<std::vector<double>> A(N, std::vector<double>(N, 0.0));

        // Initialize A based on time
        if (std::fmod(t, 2.0) >= 0 && std::fmod(t, 2.0) < 0.5) {
            A = {{0, 0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
        } else if (std::fmod(t, 2.0) >= 0.5 && std::fmod(t, 2.0) < 1.0) {
            A = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}};
        } else if (std::fmod(t, 2.0) >= 1.0 && std::fmod(t, 2.0) < 1.5) {
            A = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 1, 0}};
        } else if (std::fmod(t, 2.0) >= 1.5 && std::fmod(t, 2.0) < 2.0) {
            A = {{0, 0, 0, 1}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
        }

        for (int i = 0; i < N; ++i) {

            // std::cout << " -----------------------------------------------"<< std::endl;

            int s1 = N * n * mi + N * mi * i;
            int s2 = n * mi * i;
            std::vector<std::vector<double>> rii = {{1, 0}, {0, 1}};
            std::vector<std::vector<double>> r = {{4, -4}, {-2, 2}, {6, 4}, {-4, 4}};

            // std::vector<std::vector<double>> r = {{-4, -2}, {2, -2}, {2, -2}, {-2, -2}};

            pf[0] = (rii[0][0] + rii[0][0]) * y[s1] + (rii[0][1] + rii[1][0]) * y[s1 + 1] + r[i][0] + 2 * (y[s1] - y[s1 + 2]);
            pf[1] = (rii[1][1] + rii[1][1]) * y[s1 + 1] + (rii[0][1] + rii[1][0]) * y[s1] + r[i][1] + 2 * (y[s1 + 1] - y[s1 + 3]);

            pf[2] = (rii[0][0] + rii[0][0]) * y[s1 + 2] + (rii[0][1] + rii[1][0]) * y[s1 + 3] + r[i][0] + 2 * (y[s1 + 2] - y[s1 + 4]) + 2 * (y[s1 + 2] - y[s1]);
            pf[3] = (rii[1][1] + rii[1][1]) * y[s1 + 3] + (rii[0][1] + rii[1][0]) * y[s1 + 2] + r[i][1] + 2 * (y[s1 + 3] - y[s1 + 5]) + 2 * (y[s1 + 3] - y[s1 + 1]);

            pf[4] = (rii[0][0] + rii[0][0]) * y[s1 + 4] + (rii[0][1] + rii[1][0]) * y[s1 + 5] + r[i][0] + 2 * (y[s1 + 4] - y[s1 + 6]) + 2 * (y[s1 + 4] - y[s1 + 2]);
            pf[5] = (rii[1][1] + rii[1][1]) * y[s1 + 5] + (rii[0][1] + rii[1][0]) * y[s1 + 4] + r[i][1] + 2 * (y[s1 + 5] - y[s1 + 7]) + 2 * (y[s1 + 5] - y[s1 + 3]);   

            pf[6] = (rii[0][0] + rii[0][0]) * y[s1 + 6] + (rii[0][1] + rii[1][0]) * y[s1 + 7] + r[i][0] + 2 * (y[s1 + 6] - y[s1]);
            pf[7] = (rii[1][1] + rii[1][1]) * y[s1 + 7] + (rii[0][1] + rii[1][0]) * y[s1 + 6] + r[i][1] + 2 * (y[s1 + 7] - y[s1 + 1]);     

            u[i][0] = -delta * pf[s2];
            u[i][1] = -delta * pf[s2 + 1];

            dydt[s2] = u[i][0];
            dydt[s2 + 1] = u[i][1];

            for (int j = 0; j < N; ++j) {
                std::vector<double> m4(mi, 0.0);
                int s3 = mi * j;
                for (int k = 0; k < N; ++k) {
                    int s4 = N * n * mi + N * mi * k;
                    for (int l = 0; l < mi; ++l) {
                        m4[l] = m4[l] - alpha * A[i][k] * (y[s1 + s3 + l]-y[s4 + s3 + l]);
                    }
                }

                for (int l = 0; l < mi; ++l) {
                    dydt[s1 + s3 + l] = m4[l] - alpha * A[i][j] * (y[s1 + s3 + l]-y[mi*j+l]);
                }
            }

            w[i] =  u[i][0];
            v[i] =  u[i][1];
  
        }

        return dydt;
    }