/*
 * Copyright (C) 2012 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
package org.ros.master.client;

import java.util.Set;

/**
 * Information about a service.
 * 
 * @author Evan Krause
 */
public class ServiceSystemState {

	/**
	 * Name of the service.
	 */
	private final String serviceName;
        
	/**
	 * Node names of all service providers.
	 */
	private final Set<String> serviceProviders;

	public ServiceSystemState(String serviceName, Set<String> serviceProviders) {
		this.serviceName = serviceName;
		this.serviceProviders = serviceProviders;
	}

	/**
	 * @return the serviceName
	 */
	public String getServiceName() {
		return serviceName;
	}

	/**
	 * Get the set of all nodes that provide the service.
	 * 
	 * @return the set of node names
	 */
	public Set<String> getServiceProviders() {
		return serviceProviders;
	}
    
}
