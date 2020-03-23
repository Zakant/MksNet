using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet
{
    /// <summary>
    /// Enumeration of degrees of freedom.
    /// </summary>
    public enum DOF
    {
        /// <summary>
        /// Translation degree of freedom along the x axis.
        /// </summary>
        X,
        /// <summary>
        /// Translation degree of freedom along the y axis.
        /// </summary>
        Y,
        /// <summary>
        /// Translation degree of freedom along the z axis.
        /// </summary>
        Z,
        /// <summary>
        /// Rotational degree of freedom around the x axis.
        /// </summary>
        RotX,
        /// <summary>
        /// Rotational degree of freedom around the y axis.
        /// </summary>
        RotY,
        /// <summary>
        /// Rotational degree of freedom around the z axis.
        /// </summary>
        RotZ
    }
}
