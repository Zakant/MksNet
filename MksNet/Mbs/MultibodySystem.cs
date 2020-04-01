using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    public class MultibodySystem
    {
        /// <summary>
        /// The number of elements in the system
        /// </summary>
        public int NumberOfElements { get; private set; }

        /// <summary>
        /// The gravitational constant
        /// </summary>
        public double GravitationalConstant { get; private set; } = -9.81;
    }
}
