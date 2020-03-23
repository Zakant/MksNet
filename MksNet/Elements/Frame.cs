using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements
{
    /// <summary>
    /// Coordiante frame attached to an element.
    /// </summary>
    public class Frame
    {
        /// <summary>
        /// Offset between the element origin and the frame.
        /// Given in element coordinates.
        /// </summary>
        public Vector<double> Offset { get; internal set; }

        /// <summary>
        /// Rotation between the element origin and the frame.
        /// </summary>
        public Matrix<double> Rotation { get; internal set; }

    }
}
