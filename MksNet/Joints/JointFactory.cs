using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Joints
{
    public class JointFactory
    {
        /// <summary>
        /// Get the singelton instace of the joint factory.
        /// </summary>
        public static JointFactory Instance { get; } = new JointFactory();

        private JointFactory() { }

        /// <summary>
        /// Create a joint with the given type name.
        /// </summary>
        /// <param name="name">The name of the joint type.</param>
        /// <returns>The joint instance</returns>
        public Joint Create(string name)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Add a joint definition the the factory.
        /// </summary>
        /// <param name="definition">The definition string.</param>
        public void AddDefinition(string definition)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Load a joint definition from a file and adds it to the factory.
        /// </summary>
        /// <param name="filePath">The file path.</param>
        public void LoadDefinition(string filePath)
        {
            throw new NotImplementedException();
        }
    }
}
