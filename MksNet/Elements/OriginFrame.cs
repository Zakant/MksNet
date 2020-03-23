using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements
{
    /// <summary>
    /// Origin Frame. Does not use any other frame as an reference
    /// </summary>
    internal class OriginFrame : Frame
    {
        private static Vector<double> _offset = CreateVector.Dense<double>(3, 0);
        private static Matrix<double> _rotation = CreateMatrix.DenseIdentity<double>(3);


        /// <summary>
        /// Create a new origin frame.
        /// </summary>
        internal OriginFrame() : base(null) { }


        public override Vector<double> GetOffsetOrigin() => _offset;

        public override Matrix<double> GetRotationOrigin() => _rotation;
    }
}
