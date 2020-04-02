using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    public class MultibodySystem
    {
        /// <summary>
        /// Number of elements in the system.
        /// </summary>
        public int NumberOfElements { get; private set; }

        /// <summary>
        /// The magnitude and direction of gravitation.
        /// </summary>
        public Vector<double> GravitationVector { get; private set; }
    }
}
