using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{
    /// <summary>
    /// The exception that is thrown when the requested joint type can not be found.
    /// </summary>
    [Serializable]
    public class JointNotFoundException : Exception
    {
        public JointNotFoundException() { }
        public JointNotFoundException(string message) : base(message) { }
        public JointNotFoundException(string message, Exception inner) : base(message, inner) { }
        protected JointNotFoundException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
