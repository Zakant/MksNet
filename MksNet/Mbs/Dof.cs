using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    /// <summary>
    /// Enumeration of degrees of freedom.
    /// </summary>
    public enum Dof : byte
    {
        /// <summary>
        /// Translation degree of freedom along the x axis.
        /// </summary>
        X = 0,
        /// <summary>
        /// Translation degree of freedom along the y axis.
        /// </summary>
        Y = 1,
        /// <summary>
        /// Translation degree of freedom along the z axis.
        /// </summary>
        Z = 2,
        /// <summary>
        /// Rotational degree of freedom around the x axis.
        /// </summary>
        alpha = 3,
        /// <summary>
        /// Rotational degree of freedom around the y axis.
        /// </summary>
        beta = 4,
        /// <summary>
        /// Rotational degree of freedom around the z axis.
        /// </summary>
        gamma = 5
    }
}
