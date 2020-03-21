using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements
{
    /// <summary>
    /// Factory class to create element objects.
    /// </summary>
    public class ElementFactory
    {
        /// <summary>
        /// Get the singelton instace of the element factory.
        /// </summary>
        public static ElementFactory Instance { get; } = new ElementFactory();


        private ElementFactory() { }
        
        /// <summary>
        /// Creates an element.
        /// </summary>
        /// <param name="name">The name of the element type.</param>
        /// <returns>The element instance</returns>
        public Element Create(string name)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Add an element definition the the factory.
        /// </summary>
        /// <param name="definition">The definition string.</param>
        public void AddDefinition(string definition)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Load an element definition from a file and adds it to the factory.
        /// </summary>
        /// <param name="filePath">The file path.</param>
        public void LoadDefinition(string filePath)
        {
            throw new NotImplementedException();
        }
    }
}
