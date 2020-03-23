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

        /// <summary>
        /// Reference frame used for translation and rotation.
        /// </summary>
        public Frame Reference { get; protected set; }

        /// <summary>
        /// Creates a new frame with respect to the given reference frame.
        /// </summary>
        /// <param name="referenceFrame">Reference frame.</param>
        public Frame(Frame referenceFrame)
        {
            Reference = referenceFrame;
        }


        /// <summary>
        /// Get the rotation matrix from origin to this frame.
        /// </summary>
        /// <returns>Rotation matrix from orign to this frame.</returns>
        public virtual Matrix<double> GetRotationOrigin() => Rotation * Reference.GetRotationOrigin();

        /// <summary>
        /// Get the offset vector from origin to this frame in origin coordinates.
        /// </summary>
        /// <returns>Offset vector between this frame and origin.</returns>
        public virtual Vector<double> GetOffsetOrigin() => Reference.GetOffsetOrigin() + Reference.Rotation * Offset;
    }
}
