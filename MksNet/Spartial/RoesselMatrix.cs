using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;

namespace MksNet.Spartial
{
    static class RoesselMatrix
    {
        public static Matrix<double> GetRoesselMatrix(Vector<double> InputVector)
        {
            Matrix<double> RoesselMatrix = CreateMatrix.Dense<double>(3, 3);
            RoesselMatrix[0, 1] = -InputVector[2];
            RoesselMatrix[0, 2] =  InputVector[1];
            RoesselMatrix[1, 0] =  InputVector[2];
            RoesselMatrix[1, 2] = -InputVector[0];
            RoesselMatrix[2, 0] = -InputVector[1];
            RoesselMatrix[2, 1] =  InputVector[0];
            return RoesselMatrix;
        }
    }
}
