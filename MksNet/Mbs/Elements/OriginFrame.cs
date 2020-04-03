using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Elements
{
    /// <summary>
    /// Origin Frame. Does not use any other frame as an reference
    /// </summary>
    internal class OriginFrame : Frame
    {
        private static Vector<double> _offset = CreateVector.Dense<double>(3, 0);
        private static Matrix<double> _rotation = CreateMatrix.DenseIdentity<double>(3);

        /// <summary>
        /// Offset between the element origin and the frame.
        /// Given in element coordinates.
        /// </summary>
        public override Vector<double> Offset { get { return _offset; } internal set { } }

        /// <summary>
        /// Rotation between the element origin and the frame.
        /// </summary>
        public override Matrix<double> Rotation { get { return _rotation; } internal set { } }

        /// <summary>
        /// Name of the frame.
        /// </summary>
        public override string Name { get { return "origin"; } internal set { } }

        /// <summary>
        /// Create a new origin frame.
        /// </summary>
        internal OriginFrame() : base(null) { }


        public override Vector<double> GetOffsetOrigin() => _offset;

        public override Matrix<double> GetRotationOrigin() => _rotation;
    }
}
