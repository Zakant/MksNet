using MathNet.Numerics.LinearAlgebra;
using MksNet.Joints;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements
{
    /// <summary>
    /// Represents on element of a multibody system.
    /// </summary>
    public sealed class Element
    {
        /// <summary>
        /// Id of the element.
        /// </summary>
        public int ElementId { get; internal set; }

        /// <summary>
        /// System the element belongs to.
        /// </summary>
        public MultibodySystem System { get; internal set; }

        /// <summary>
        /// Read-only collection of all available frames.
        /// </summary>
        public IReadOnlyCollection<Frame> Frames { get; internal set; }

        /// <summary>
        /// Mass of the element.
        /// </summary>
        public double Mass { get; internal set; }

        /// <summary>
        /// Inertia of the element with respect to element coordinates.
        /// </summary>
        public Matrix<double> Inertia { get; internal set; }

        /// <summary>
        /// Base joint this element follows. Determines the degrees of freedom.
        /// </summary>
        public Joint BaseJoint { get; internal set; }

        internal Element()
        { }

        public Matrix<double> GetJacobian()
        {
            throw new NotImplementedException();
        }

        public Matrix<double> GetJacobianDerivativ()
        {
            throw new NotImplementedException();
        }

    }
}
