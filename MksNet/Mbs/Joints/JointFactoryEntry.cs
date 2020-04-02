using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs.Joints
{
    internal class JointFactoryEntry
    {

        internal string Name;

        internal string Author;

        internal string Description;

        internal string Url;

#pragma warning disable CS0649
        internal List<Dof> LockedDofs;

        internal List<Dof> FreeDofs;
#pragma warning restore CS0649
    }
}
