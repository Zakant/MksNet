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
        public int ElementId { get; private set; }

        /// <summary>
        /// System the element belongs to.
        /// </summary>
        public MultibodySystem System { get; private set; }

        /// <summary>
        /// Parent element.
        /// </summary>
        public Element Parent { get; private set; }
    }
}
