using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Differentiation;

namespace ik_test
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
            for (int i = 3; i < 6; i++)
            {
                double[] goal = new double[]{0,0,0,0,0,0};
                goal[i]=1;
                iterative_inverse_kinematics(goal);
            }
        }

        public static void iterative_inverse_kinematics(double[] distance)
        {
            //initalize velocity vectory based on distance and time, using arbitrary time step of 10 seconds
            double t = 10;
            double[] xp = new double[6];
            
            for (int i = 0; i < distance.Length; i++)
            {
                xp[i] = (double)(distance[i] / t);
            }
            MathNet.Numerics.LinearAlgebra.Vector<double> xpVec = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(xp);
//           //iterations
            int iterationCount = 10001;
            double timeStep = t / iterationCount;
            //initalize position and velocity arrays 
            MathNet.Numerics.LinearAlgebra.Vector<double> currentPosition;
            MathNet.Numerics.LinearAlgebra.Vector<double> nextPosition;
            MathNet.Numerics.LinearAlgebra.Vector<double> currentVelocity;
            MathNet.Numerics.LinearAlgebra.Vector<double> nextVelocity;
//           //set initial position
            double[] startingPos = new double[] { 0, -1.57f, 0, 1.57f, 0, 1.57f, 0 };
            
            //convert to vector
            currentPosition = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(startingPos);
///           //iterate over timesteps
            for (int i = 0;i<iterationCount;i++)
            {
                //get jacobian matrix
                Matrix<double> jac = jacobian(currentPosition);
                //calculate joint velocities from psuedo inverse
                nextVelocity = jac.PseudoInverse() * xpVec;
                //calculate the next step position based on joint velocities
                nextPosition = currentPosition + nextVelocity * timeStep;
                // setup for next step;
                currentPosition = nextPosition;
                currentVelocity = nextVelocity;
            }
            double[] finalPosition = new double[7];
            for (int i = 0; i < currentPosition.Count; i++)
            {
                finalPosition[i] = currentPosition[i];
            }
            double[] degreesOut = convertToDegrees(finalPosition);
            Console.WriteLine(string.Format("Goal was: {0},{1},{2},{3},{4},{5}",distance[0],distance[1],distance[2],distance[3],distance[4],distance[5]));
            Console.WriteLine(string.Format("End angles are {0},{1},{2},{3},{4},{5},{6}",degreesOut[0],degreesOut[1],degreesOut[2],degreesOut[3],degreesOut[4],degreesOut[5],degreesOut[6]));
        }
        public static double[] convertToDegrees(double[] radiansIn)
        {
            double[] degreesOut = new double[7];
            for(int i = 0; i<radiansIn.Length;i++)
            {
                degreesOut[i] = Math.Round(radiansIn[i] * 57.2958);
            }
            return degreesOut;
        }
        public static Matrix<double> jacobian(MathNet.Numerics.LinearAlgebra.Vector<double> jointAngles, bool gripper = true, int dof = 7)
        {
            double q1 = jointAngles[0];
            double q2 = jointAngles[1];
            double q3 = jointAngles[2];
            double q4 = jointAngles[3];
            double q5 = jointAngles[4];
            double q6 = jointAngles[5];
            if (dof == 7)
            {
                double q7 = jointAngles[6];
            }
            double x0 = Math.Cos(q1);
            double x1 = Math.Sin(q1);
            double x2;
            if (dof == 6) { x2 = Math.Cos(q3); } else { x2 = Math.Sin(q2); }
            double x3;
            if (dof == 6) { x3 = Math.Sin(q2); } else { x3 = 0.4208f * x2; }
            double x4;
            if (dof == 6) { x4 = x2 * x3; } else { x4 = Math.Cos(q3); }
            double x5;
            if (dof == 6) { x5 = x1 * x4; } else { x5 = x0 * x4; }
            double x6;
            if (dof == 6) { x6 = Math.Cos(q2); } else { x6 = 0.0128f * x5; }
            double x7;
            if (dof == 6) { x7 = Math.Sin(q3); } else { x7 = Math.Cos(q4); }
            double x8;
            if (dof == 6) { x8 = x6 * x7; } else { x8 = x1 * x2; }
            double x9;
            if (dof == 6) { x9 = x1 * x8; } else { x9 = x7 * x8; }
            double x10;
            if (dof == 6) { x10 = Math.Cos(q5); } else { x10 = Math.Cos(q2); }
            double x11;
            if (dof == 6) { if (gripper) { x11 = -0.0756f * x5 + 0.0756f * x9; } else { x11 = 0.0444f * x5 - 0.0555f * x9; } } else { x11 = Math.Sin(q3); }
            double x12;
            if (dof == 6) { x12 = Math.Sin(q5); } else { x12 = x1 * x11; }
            double x13;
            if (dof == 6) { x13 = Math.Sin(q4); } else { x13 = x10 * x12; }
            double x14;
            if (dof == 6) { x14 = x0 * x13; } else { x14 = Math.Sin(q4); }
            double x15;
            if (dof == 6) { x15 = x3 * x7; } else { x15 = x0 * x11; }
            double x16;
            if (dof == 6) { x16 = x1 * x15; } else { x16 = 0.3143f * x15; }
            double x17;
            if (dof == 6) { x17 = x2 * x6; } else { x17 = x1 * x4; }
            double x18;
            if (dof == 6) { x18 = x1 * x17; } else { x18 = x10 * x17; }
            double x19;
            if (dof == 6) { x19 = -x17 - x18; } else { x19 = -x16 - 0.3143f * x18; }
            double x20;
            if (dof == 6) { x20 = Math.Cos(q4); } else { x20 = Math.Cos(q6); }
            double x21;
            if (dof == 6) { if (gripper) { x21 = 0.0756f * x20; } else { x21 = 0.0444f * x20; } } else { if (gripper) { x21 = -0.2874f * x9; } else { x21 = -0.1674f * x9; } }
            double x22;
            if (dof == 6) { if (gripper) { x22 = 0.0756f * x14 - x19 * x21; } else { x22 = -0.0444f * x14 + x19 * x21; } } else { x22 = x15 + x18; }
            double x23;
            if (dof == 6) { x23 = x0 * x15; } else { x23 = x14 * x22; }
            double x24;
            if (dof == 6) { x24 = 0.1025f * x23; } else { x24 = Math.Sin(q6); }
            double x25;
            if (dof == 6) { x25 = x0 * x17; } else { x25 = x13 - x5; }
            double x26;
            if (dof == 6) { x26 = 0.1025f * x25; } else { x26 = Math.Sin(q5); }
            double x27;
            if (dof == 6) { if (gripper) { x27 = 0.0756f * x23; } else { x27 = 0.0444f * x23; } } else { if (gripper) { x27 = 0.2874f * x26; } else { x27 = 0.1674f * x26; } }
            double x28;
            if (dof == 6) { if (gripper) { x28 = 0.0756f * x25; } else { x28 = 0.0444f * x25; } } else { x28 = x14 * x2; }
            double x29;
            if (dof == 6) { x29 = x0 * x8; } else { x29 = x1 * x28; }
            double x30;
            if (dof == 6) { x30 = x0 * x4; } else { x30 = -x15 - x18; }
            double x31;
            if (dof == 6) { x31 = x12 * x21; } else { x31 = x29 + x30 * x7; }
            double x32;
            if (dof == 6) { x32 = x1 * x20; } else { x32 = Math.Cos(q5); }
            double x33;
            if (dof == 6) { x33 = x23 + x25; } else { if (gripper) { x33 = 0.2874f * x32; } else { x33 = 0.1674f * x32; } }
            double x34;
            if (dof == 6) { if (gripper) { x34 = 0.0756f * x13; } else { x34 = 0.0444f * x13; } } else { x34 = x31 * x33; }
            double x35;
            if (dof == 6) { if (gripper) { x35 = 0.0756f * x30; } else { x35 = 0.0444f * x30; } } else { x35 = 0.4208f * x10; }
            double x36;
            if (dof == 6) { if (gripper) { x36 = 0.0756f * x29; } else { x36 = 0.0444f * x29; } } else { x36 = 0.0128f * x15; }
            double x37;
            if (dof == 6) { x37 = x1 * x13; } else { x37 = x10 * x7; }
            double x38;
            if (dof == 6) { if (gripper) { x38 = 0.0756f * x37; } else { x38 = 0.0444f * x37; } } else { x38 = 0.3143f * x37; }
            double x39;
            if (dof == 6) { x39 = 0.1025f * x16; } else { x39 = 0.3143f * x5; }
            double x40;
            if (dof == 6) { x40 = 0.1025f * x18; } else { if (gripper) { x40 = 0.2874f * x37; } else { x40 = 0.1674f * x37; } }
            double x41;
            if (dof == 6) { if (gripper) { x41 = 0.0756f * x16; } else { x41 = 0.0444f * x16; } } else { if (gripper) { x41 = 0.2874f * x28; } else { x41 = 0.1674f * x28; } }
            double x42;
            if (dof == 6) { if (gripper) { x42 = 0.0756f * x18; } else { x42 = 0.0444f * x18; } } else { x42 = x2 * x27; }
            double x43;
            if (dof == 6) { x43 = x0 * x20; } else { x43 = x10 * x14; }
            double x44;
            if (dof == 6) { x44 = 0.1025f * x4; } else { x44 = x2 * x7; }
            double x45;
            if (dof == 6) { x45 = 0.1025f * x8; } else { x45 = 0.0128f * x12; }
            double x46;
            if (dof == 6) { if (gripper) { x46 = 0.0756f * x4; } else { x46 = 0.0444f * x4; } } else { x46 = 0.3143f * x17; }
            double x47;
            if (dof == 6) { if (gripper) { x47 = 0.0756f * x8; } else { x47 = 0.0444f * x8; } } else { x47 = x10 * x15; }
            double x48;
            if (dof == 6) { if (gripper) { x48 = 0.0756f * x15; } else { x48 = 0.0444f * x15; } } else { x48 = x17 + x47; }
            double x49;
            if (dof == 6) { if (gripper) { x49 = 0.0756f * x17; } else { x49 = 0.0444f * x17; } } else { if (gripper) { x49 = 0.2874f * x20; } else { x49 = 0.1674f * x20; } }
            double x50;
            if (dof == 6) { x50 = x12 * (-x48 - x49); } else { x50 = x14 * x49; }
            double x51;
            if (dof == 6) { if (gripper) { x51 = x46 - x47; } else { x51 = -x46 + x47; } } else { x51 = x10 * x5; }
            double x52;
            if (dof == 6) { x52 = x10 * x51; } else { x52 = x12 - x51; }
            double x53;
            if (dof == 6) { x53 = x12 * x20; } else { x53 = x33 * (-x17 - x47); }
            double x54;
            if (dof == 6) { x54 = q2 - q3; } else { x54 = x0 * x28; }
            double x55;
            if (dof == 6) { x55 = Math.Sin(x54); } else { x55 = 0.3143f * x12; }
            double x56;
            if (dof == 6) { x56 = x0 * x55; } else { x56 = 0.3143f * x51; }
            double x57;
            if (dof == 6) { x57 = Math.Cos(x54); } else { x57 = x52 * x7; }
            double x58;
            if (dof == 6) { x58 = x1 * x55; } else { x58 = x0 * x2; }
            double x59 = x58 * x7;
            double x60 = -x12 + x51;
            double x61 = x14 * x60;
            double x62 = -x59 - x61;
            double x63 = x24 * x33;
            double x64 = -x54 + x60 * x7;
            double x65 = x26 * x64;
            double x66;
            if (gripper) { x66 = 0.2874f * x59; } else { x66 = 0.1674f * x59; }
            double x67;
            if (gripper) { x67 = 0.2874f * x61; } else { x67 = 0.1674f * x61; }
            double x68 = x26 * x48;
            double x69;
            if (gripper) { x69 = 0.2874f * x68; } else { x69 = 0.1674f * x68; }
            double x70 = x32 * x64;
            double x71 = -x13 + x5;
            double x72 = x25 * x33;
            double x73 = x22 * x7;
            double x74 = x14 * x30;
            double x75 = x26 * x71;
            double x76 = 0.3143f * x44;
            double x77 = x10 * x11;
            double x78 = 0.3143f * x43;
            double x79;
            if (gripper) { x79 = 0.2874f * x44; } else { x79 = 0.1674f * x44; }
            double x80;
            if (gripper) { x80 = 0.2874f * x43; } else { x80 = 0.1674f * x43; }
            double x81 = x2 * x4;
            double x82 = x11 * x28;
            double x83 = x11 * x2;
            double x84 = x33 * x83;
            double x85 = x28 * x4;
            double x86 = -x37 + x85;
            double x87 = x4 * x44;
            double x88 = -x43 - x87;
            double x89 = x23 + x9;
            double x90 = x29 - x73;
            double x91 = x43 + x87;
            double[][] matrixArray = new double[6][];
            if (dof == 6)
            {
                if (gripper) 
                {
                    matrixArray[0] = new double[] 
                    { 
                        -0.409f * x0 + x10 * x11 + x12 * x22 + 0.1025f * x5 - 0.1025f * x9, 
                        x10 * (x27 + x28) - x24 - x26 - x31 * (x29 - x30), 
                        x10 * (-x27 - x28) + x24 + x26 - x31 * (-x29 + x30), 
                        x12 * (0.0756f * x32 + x33 * x34), 
                        x10 * (-x21 * x33 + x38) - x12 * (x35 - x36), 
                        0f 
                    };
                    matrixArray[1] = new double[] 
                    { 
                        0.409f * x1 + x10 * (-x35 + x36) + x12 * (-x21 * (-x23 - x25) - x38) - 0.1025f * x29 + 0.1025f * x30, 
                        x10 * (-x41 - x42) - x31 * (x5 - x9) + x39 + x40, 
                        x10 * (x41 + x42) - x31 * (-x5 + x9) - x39 - x40, 
                        x12 * (x19 * x34 + 0.0756f * x43), 
                        x10 * x22 - x11 * x12, 
                        0f 
                    };
                    matrixArray[2] = new double[]
                    { 
                        0, 
                        x10 * (-x46 + x47) + x20 * x50 + x44 - x45, 
                        -x44 + x45 + x52 + x53 * (-x48 - x49), 
                        -x12 * x13 * x51, 
                        x20 * x52 - x50, 
                        0f 
                    };
                    matrixArray[3] = new double[] 
                    { 
                        0, 
                        x1, 
                        -x1, 
                        -x56, 
                        -x14 * x57 - x32, 
                        -x10 * x56 + x12 * (-x37 + x43 * x57)
                    };
                    matrixArray[4] = new double[] 
                    { 
                        0, 
                        x0, 
                        -x0, 
                        x58, 
                        x37 * x57 - x43, 
                        x10 * x58 - x12 * (x14 + x32 * x57) 
                    };
                    matrixArray[5] = new double[] 
                    { 
                        -1f,
                        0f, 
                        0f, 
                        -x57, 
                        x13 * x55,
                        -x10 * x57 - x53 * x55 
                    };
                }
                else
                {
                    matrixArray[0] = new double[] 
                    { 
                        -0.409f * x0 + x10 * x11 + x12 * x22 + 0.1025f * x5 - 0.1025f * x9, 
                        x10 * (-x27 - x28) - x24 - x26 + x31 * (x29 - x30), 
                        x10 * (x27 + x28) + x24 + x26 + x31 * (-x29 + x30), 
                        x12 * (-0.0444f * x32 - x33 * x34), 
                        x10 * (x21 * x33 - x38) - x12 * (-x35 + x36), 
                        0f
                    };
                    matrixArray[1] = new double[] 
                    { 
                        0.409f * x1 + x10 * (x35 - x36) + x12 * (x21 * (-x23 - x25) + x38) - 0.1025f * x29 + 0.1025f * x30, 
                        x10 * (x41 + x42) + x31 * (x5 - x9) + x39 + x40, 
                        x10 * (-x41 - x42) + x31 * (-x5 + x9) - x39 - x40, 
                        x12 * (-x19 * x34 - 0.0444f * x43), 
                        x10 * x22 - x11 * x12, 
                        0f 
                    };
                    matrixArray[2] = new double[] 
                    { 
                        0f, 
                        x10 * (x46 - x47) + x20 * x50 + x44 - x45, 
                        -x44 + x45 + x52 + x53 * (x48 + x49), 
                        -x12 * x13 * x51, 
                        x20* x52 -x50, 
                        0f 
                    };
                    matrixArray[3] = new double[] 
                    { 
                        0f, 
                        x1, 
                        -x1, 
                        -x56, 
                        -x14 * x57 - x32, 
                        -x10 * x56 + x12 * (-x37 + x43 * x57) 
                    };
                    matrixArray[4] = new double[] 
                    { 
                        0f, 
                        x0, 
                        -x0, 
                        x58, 
                        x37 * x57 - x43, 
                        x10 * x58 - x12 * (x14 + x32 * x57) 
                    };
                    matrixArray[5] = new double[] 
                    { 
                        -1f, 
                        0f, 
                        0f, 
                        -x57, 
                        x13 * x55, 
                        -x10 * x57 - x53 * x55 
                    };
                }
            }
            else
            {
                if (gripper)
                {
                    matrixArray[0] = new double[]
                    { 
                        -0.0118f * x0 - x1 * x3 + 0.0128f * x13 + x14 * x19 + x20 * (x21 - 0.2874f * x23) + x24 * (x25 * x27 + x34) - x6 - 0.3143f * x9, 
                        x0* x35 +x0 * x38 + x2 * x36 + x20 * (x0 * x40 - x41 * x5) + x24 * (x15 * x42 + x33 * (-x0 * x43 - x44 * x5)) - x28 * x39, 
                        -x10 * x6 + x14 * (-x10 * x16 - x46) + x24 * (x27 * x52 + x53 * x7) + x45 - x48 * x50, 
                        x20 * (-0.2874f * x54 - 0.2874f * x57) - 0.3143f * x54 + x62 * x63 + x7 * (-x55 + x56), 
                        x24 * (x53 - 0.2874f * x65), 
                        x20 * (-x69 + 0.2874f * x70) - x24 * (x66 + x67), 
                        0
                    };
                    matrixArray[1] = new double[]
                    { 
                        -x0 * x3 + 0.0118f * x1 + x14 * (x55 - x56) + 0.0128f * x17 + x20 * (-x66 - x67) + x24 * (x33 * (x54 + x57) + x69) + 0.0128f * x47 - 0.3143f * x59, 
                        -x1 * x35 - x1 * x38 - x2 * x45 + x20 * (-x1 * x40 + x17 * x41) + x24 * (-x12 * x42 + x33 * (x1 * x43 + x17 * x44)) + x28 * x46, 
                        x14 * (0.3143f * x13 - x39) + 0.0128f * x18 + x24 * (x22 * x27 + x7 * x72) + x36 - x50 * x71, 
                        x19* x7 +x20 * (0.2874f * x29 - 0.2874f * x73) + 0.3143f * x29 + x63 * (-x74 + x9), 
                        x24 * (-x27 * x31 + x72), 
                        x20 * (x34 - 0.2874f * x75) - x24 * (x21 + 0.2874f * x74), 
                        0
                    };
                    matrixArray[2] = new double[]
                    { 
                        0, 
                        x20 * (-x4 * x80 - x79) + x24 * (x27 * x77 + x33 * (x28 - x37 * x4)) - x3 - x4 * x78 - x76 + 0.0128f * x77, 
                        x24 * (x27 * x81 + x7 * x84) + x49 * x82 + 0.0128f * x81 + 0.3143f * x82, 
                        x20 * (-x4 * x79 - x80) - x4 * x76 + x63 * x86 - x78, 
                        x24 * (-x27 * x88 + x84), 
                        x20 * (x27 * x83 + x33 * x88) - x24 * (x40 - 0.2874f * x85), 
                        0
                    };
                    matrixArray[3] = new double[]
                    { 
                        0f, 
                        x1, 
                        -x58, 
                        x48, 
                        x62, 
                        x32* x48 +x65,
                        x20* x62 -x24 * (-x68 + x70)
                    };
                    matrixArray[4] = new double[]
                    { 
                        0f, 
                        x0, 
                        x8, 
                        x71, 
                        x89, 
                        x26* x90 +x32 * x71, 
                        x20* x89 -x24 * (x32 * x90 - x75)
                    };
                    matrixArray[5] = new double[]
                    { 
                        -1f, 
                        0f, 
                        -x10, 
                        -x83, 
                        x86, 
                        -x26 * x91 - x32 * x83, 
                        x20* x86 -x24 * (x26 * x83 - x32 * x91)
                    };
                }
                else
                {
                    matrixArray[0] = new double[]
                    {
                        -0.0118f * x0 - x1 * x3 + 0.0128f * x13 + x14 * x19 + x20 * (x21 - 0.1674f * x23) + x24 * (x25 * x27 + x34) - x6 - 0.3143f * x9, 
                        x0 * x35 + x0 * x38 + x2 * x36 + x20 * (x0 * x40 - x41 * x5) + x24 * (x15 * x42 + x33 * (-x0 * x43 - x44 * x5)) - x28 * x39, 
                        -x10 * x6 + x14 * (-x10 * x16 - x46) + x24 * (x27 * x52 + x53 * x7) + x45 - x48 * x50, 
                        x20 * (-0.1674f * x54 - 0.1674f * x57) - 0.3143f * x54 + x62 * x63 + x7 * (-x55 + x56), 
                        x24 * (x53 - 0.1674f * x65), 
                        x20 * (-x69 + 0.1674f * x70) - x24 * (x66 + x67), 
                        0f
                    };
                    matrixArray[1] = new double[]
                    {
                        -x0 * x3 + 0.0118f * x1 + x14 * (x55 - x56) + 0.0128f * x17 + x20 * (-x66 - x67) + x24 * (x33 * (x54 + x57) + x69) + 0.0128f * x47 - 0.3143f * x59, 
                        -x1 * x35 - x1 * x38 - x2 * x45 + x20 * (-x1 * x40 + x17 * x41) + x24 * (-x12 * x42 + x33 * (x1 * x43 + x17 * x44)) + x28 * x46, 
                        x14 * (0.3143f * x13 - x39) + 0.0128f * x18 + x24 * (x22 * x27 + x7 * x72) + x36 - x50 * x71, 
                        x19* x7 +x20 * (0.1674f * x29 - 0.1674f * x73) + 0.3143f * x29 + x63 * (-x74 + x9), 
                        x24 * (-x27 * x31 + x72), 
                        x20 * (x34 - 0.1674f * x75) - x24 * (x21 + 0.1674f * x74), 
                        0f
                    };
                    matrixArray[2] = new double[]
                    {
                        0f, 
                        x20 * (-x4 * x80 - x79) + x24 * (x27 * x77 + x33 * (x28 - x37 * x4)) - x3 - x4 * x78 - x76 + 0.0128f * x77, 
                        x24 * (x27 * x81 + x7 * x84) + x49 * x82 + 0.0128f * x81 + 0.3143f * x82, 
                        x20 * (-x4 * x79 - x80) - x4 * x76 + x63 * x86 - x78, 
                        x24 * (-x27 * x88 + x84),
                        x20 * (x27 * x83 + x33 * x88) - x24 * (x40 - 0.1674f * x85), 
                        0f
                    };
                    matrixArray[3] = new double[]
                    {
                        0f, 
                        x1, 
                        -x58, 
                        x48, 
                        x62, 
                        x32 * x48 + x65, 
                        x20 * x62 - x24 * (-x68 + x70) 
                    };
                    matrixArray[4] = new double[]
                    {
                        0f, 
                        x0, 
                        x8, 
                        x71, 
                        x89, 
                        x26* x90 +x32 * x71, 
                        x20* x89 -x24 * (x32 * x90 - x75)
                    };
                    matrixArray[5] = new double[]
                    {
                        -1f, 
                        0f, 
                        -x10, 
                        -x83, 
                        x86, 
                        -x26 * x91 - x32 * x83, 
                        x20* x86 -x24 * (x26 * x83 - x32 * x91)
                    };
                }
            }
            Matrix<double> matrixOut = Matrix<double>.Build.DenseOfRowArrays(matrixArray);
            return matrixOut;
        }
    }
}
