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
        /// Parent element.
        /// </summary>
        public Element Parent { get; internal set; }

        /// <summary>
        /// All frames attached to the element.
        /// </summary>
        public IReadOnlyList<Frame> Frames { get; internal set; }

        /// <summary>
        /// Origin frame of the element.
        /// </summary>
        public Frame Origin { get; internal set; }
    }
}
