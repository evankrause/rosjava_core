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
package org.ros.internal.node.service;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import java.io.IOException;
import java.net.Socket;
import java.net.URI;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.Map;
import org.ros.internal.node.client.MasterClient;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;

/**
 * Provides way to query running service for its service type.
 * 
 * @author Evan Krause
 */
public class ServiceTypeClient {

    public static String getServiceType(final String serviceName, final MasterClient master) {
        // Get service URI
        //MasterClient master = new MasterClient(NodeConfiguration.newPrivate().getMasterUri());
        URI serviceURI = master.lookupService(GraphName.root(), serviceName).getResult();

        // Create the socket connection
        Socket s = null;
        try {
            s = new Socket(serviceURI.getHost(), serviceURI.getPort());
        } catch (UnknownHostException uhe) {
            // Server Host unreachable 
            System.err.println("Unknown Host:" + serviceURI.toString());
            s = null;
        } catch (IOException ioe) {
            // Cannot connect to port on given server host  
            System.err.println("Can't connect to server:" + serviceURI.toString());
            s = null;
        }

        if (s == null) {
            return "";
        }

        String serviceType = "";
        try {
            // Construct header to probe running service
            ByteBuffer outgoingHeader = buildHeader(serviceName);

            // Write header to socket
            s.getOutputStream().write(outgoingHeader.array(), 0, outgoingHeader.position());
            s.getOutputStream().flush();

            // Read from socket and decode header
            byte[] incomingHeader = new byte[2048];
            s.getInputStream().read(incomingHeader);

            // Decode incoming header
            Map<String, String> fields = decodeHeader(incomingHeader);
            if (fields.containsKey("type")) {
                serviceType = fields.get("type");
            } else {
                System.err.println("Returned header does not contain \"type\" field.");
            }
        } catch (IOException ioe) {
            System.err.println("Exception during communication. Server probably closed connection.");
        } finally {
            try {
                // Close the socket before quitting 
                s.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return serviceType;
    }

    private static ByteBuffer buildHeader(final String serviceName) {
        Map<String, String> fields = Maps.newHashMap();
        fields.put("probe", "1");
        fields.put("md5sum", "*");
        fields.put("callerid", "/rosservice");
        fields.put("service", serviceName);

        ByteBuffer buffer = ByteBuffer.allocate(2048);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putInt(0); //4 byte place holder for header size
        int headerSize = 0;
        for (Map.Entry<String, String> entry : fields.entrySet()) {
            String field = entry.getKey() + "=" + entry.getValue();
            buffer.putInt(field.length());
            buffer.put(field.getBytes(Charset.forName("US-ASCII")));
            headerSize += (4 + field.length());
        }
        buffer.putInt(0, headerSize); //header size at beginning of header
        return buffer;
    }

    public static Map<String, String> decodeHeader(byte[] header) {
        // Create byte array with input bytes
        ByteBuffer buffer = ByteBuffer.wrap(header);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        // Fill map with key/val pairs
        Map<String, String> fields = Maps.newHashMap();
        int readableBytes = buffer.getInt() + 4; //+4 for header size
        int position = 4; //start after int header size
        while (position < readableBytes) {
            // Get and check next field size
            int fieldSize = buffer.getInt();
            position += 4;
            if (fieldSize == 0) {
                throw new IllegalStateException("Invalid 0 length handshake header field.");
            }
            if (position + fieldSize > readableBytes) {
                throw new IllegalStateException("Invalid line length handshake header field.");
            }

            // Get field as "key=value" String
            byte[] tmp = new byte[fieldSize];
            buffer.get(tmp, 0, fieldSize);
            String field = new String(tmp, Charset.forName("UTF-8"));
            position += field.length();

            // Extract key and value
            Preconditions.checkState(field.indexOf("=") > 0,
                    String.format("Invalid field in handshake header: \"%s\"", field));
            String[] keyAndValue = field.split("=");
            if (keyAndValue.length == 1) {
                fields.put(keyAndValue[0], "");
            } else {
                fields.put(keyAndValue[0], keyAndValue[1]);
            }
        }
        return fields;
    }
}
