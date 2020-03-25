using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{
    /// <summary>
    /// The exception that is thrown when the joint type allready exists within the factory.
    /// </summary>
    [Serializable]
    public class JointAllreadyExistsException : Exception
    {
        public JointAllreadyExistsException() { }
        public JointAllreadyExistsException(string message) : base(message) { }
        public JointAllreadyExistsException(string message, Exception inner) : base(message, inner) { }
        protected JointAllreadyExistsException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
