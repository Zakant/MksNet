using MksNet.Elements;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Joints
{
    /// <summary>
    /// Represents a joint to connect two elements.
    /// </summary>
    public class Joint
    {
        /// <summary>
        /// Base frame the joint connects to.
        /// </summary>
        public Frame BaseFrame { get; internal set; }

        /// <summary>
        /// Follower frame the joint connects to.
        /// </summary>
        public Frame FollowerFrame { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public IEnumerable<DOF> LockedDegreesOfFreedom { get; internal set; }

        internal Joint()
        { }
    }
}
