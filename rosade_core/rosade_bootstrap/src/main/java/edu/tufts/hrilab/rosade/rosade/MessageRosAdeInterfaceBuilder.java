package edu.tufts.hrilab.rosade;

import com.google.common.base.Preconditions;
import com.google.common.collect.Sets;
import org.apache.commons.lang.StringEscapeUtils;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageInterfaceBuilder;
import org.ros.internal.message.context.MessageContext;
import org.ros.internal.message.context.MessageContextProvider;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.FieldType;
import org.ros.internal.message.field.MessageFields;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageFactory;

import java.util.Set;

/**
 * Modified from the rosjava code originally written by D. Kohler.
 * <p/>
 * TODO: (jvia) refactor this crazines
 *
 * @author damonkohler@google.com (Damon Kohler)
 * @author Jeremiah Via <jeremiah.via@gmail.com>
 */
public class MessageRosAdeInterfaceBuilder extends MessageInterfaceBuilder {

  private static String escapeJava(String str) {
    return StringEscapeUtils.escapeJava(str).replace("\\/", "/").replace("'", "\\'");
  }

  public String build(MessageFactory messageFactory) {
    Preconditions.checkNotNull(getMessageDeclaration());
    Preconditions.checkNotNull(getInterfaceName());
    StringBuilder builder = new StringBuilder();
    if (getPackageName() != null) {
      builder.append(String.format("package edu.tufts.hrilab.rosade.msg.%s;\n\n", getPackageName()));
    }
    builder.append("import java.io.Serializable;\nimport org.ros.node.ConnectedNode;\nimport org.ros.node.topic.Publisher;\n\n");
    builder.append(String.format(
            "public class %s implements Serializable {\n", getInterfaceName()));
    builder.append(String.format("  static final java.lang.String _TYPE = \"%s\";\n",
            getMessageDeclaration().getType()));
    builder.append(String.format("  static final java.lang.String _DEFINITION = \"%s\";\n",
            escapeJava(getMessageDeclaration().getDefinition())));
    if (getAddConstantsAndMethods()) {
      MessageContextProvider messageContextProvider = new MessageContextProvider(messageFactory);
      MessageContext messageContext = messageContextProvider.get(getMessageDeclaration());

//            System.out.println(messageContext.getName());
      // Add member fields (both constant and mutable)
      appendConstants(messageContext, builder);
      appendFields(messageContext, builder);

      // Methods
      appendConstructor(getInterfaceName(), messageContext, builder);
      appendSettersAndGetters(messageContext, builder);
      appendRosToAdeConverter(messageContext, builder);
      appendRosToAdeCopyConverter(messageContext, builder);
      appendAdeToRosConverter(messageContext, builder);
    }

    if (getNestedContent() != null) {
      builder.append("\n");
      builder.append(getNestedContent());
    }
    builder.append("}\n");
    return builder.toString();
  }

  private void appendConstants(MessageContext messageContext, StringBuilder builder) {
    MessageFields messageFields = new MessageFields(messageContext);
    for (Field field : messageFields.getFields()) {
      if (field.isConstant()) {
        Preconditions.checkState(field.getType() instanceof PrimitiveFieldType);
                // We use FieldType and cast back to PrimitiveFieldType below to avoid a
        // bug in the Sun JDK: http://gs.sun.com/view_bug.do?bug_id=6522780
        FieldType fieldType = (FieldType) field.getType();
        String value = getJavaValue((PrimitiveFieldType) fieldType, field.getValue().toString());
        builder.append(String.format("  public static final %s %s = %s;\n",
                fieldType.getJavaTypeName(),
                field.getName(), value));
      }
    }
    builder.append("\n");
  }

  private void appendFields(MessageContext messageContext, StringBuilder builder) {
    MessageFields messageFields = new MessageFields(messageContext);
    for (Field field : messageFields.getFields()) {
      FieldType fieldType = (FieldType) field.getType();
      if (field.isConstant()) {
        continue;
      }
      if (needsInitializer(field)) {
        builder.append(String.format("  private %s __%s = new %s();\n",
                determineType(field), determineName(field), determineType(field).replaceFirst("java.util.List", "java.util.ArrayList")));
      } else {
        builder.append(String.format("  private %s __%s;\n",
                determineType(field), determineName(field)));
      }
    }
    builder.append("\n");
  }

  private boolean needsInitializer(Field field) {
    if (field.getJavaTypeName().equals("java.lang.String")) {
      return true;
    }
    return !isPrimitiveType(field.getJavaTypeName());
  }

  private void appendConstructor(String className, MessageContext messageContext, StringBuilder builder) {
    MessageFields messageFields = new MessageFields(messageContext);

    // empty constructor for use with copying (only necessary for messages that have data)
    if (!hasOnlyConstants(messageFields)) {
      builder.append(String.format("  public %s(){\n  }\n\n", className));
    }
    // regular constructor
    builder.append(String.format("  public %s(", className));

    // Arguments
    for (Field field : messageFields.getFields()) {
      if (!(field.isConstant() || isUnsupported(field))) {
        builder.append(String.format("%s _%s, ", determineType(field), determineName(field)));
      }
    }

    if (builder.lastIndexOf("(") != builder.length() - 1) {
      builder.delete(builder.length() - 2, builder.length());
    }
    builder.append(") {\n");

    // Assignment
    for (Field field : messageFields.getFields()) {
      if (!(field.isConstant() || isUnsupported(field))) {
        builder.append(String.format("    this.__%s = _%s;\n", determineName(field), determineName(field)));
      }
    }
    builder.append("  }\n\n");

  }

  /**
   * Converts an ADE message back to the ROS version. This is slightly more
   * complex because ROS messages cannot be directly instantiated.
   * <p/>
   * TODO: (jvia) Work out to better make ROS messages on the fly. This seems
   * inefficient.
   *
   * @param messageContext the parsed ROs message
   * @param builder the file so far
   */
  private void appendAdeToRosConverter(MessageContext messageContext, StringBuilder builder) {
    String type = getPackageName() + "." + getInterfaceName();
    builder.append(String.format("  public static %s.%s toRos(edu.tufts.hrilab.rosade.msg.%s.%s val, ConnectedNode node) {\n", getPackageName(), getInterfaceName(),
            getPackageName(), getInterfaceName()));
    builder.append(String.format("    %s.%s msg = node.getTopicMessageFactory().newFromType(_TYPE);\n", getPackageName(), getInterfaceName()));
    MessageFields fields = new MessageFields(messageContext);
    for (Field field : fields.getFields()) {
      if (field.isConstant()) {
        continue;
      }
      // Headers are special
      if (getInterfaceName().equals("Header") && field.getName().equals("stamp")) {
        continue;
      }

      String setter = messageContext.getFieldSetterName(field.getName());
      String getter = messageContext.getFieldGetterName(field.getName());
      String javaType = determineType(field);
      if (isPrimitiveType(determineType(field))) {
        builder.append(String.format("    msg.%s(val.%s());\n", setter, getter));
      } else if (isContainer(field)) {
        String list = "cpy_" + field.getName().toLowerCase() + "List";
        String containedType = field.getJavaTypeName().replaceFirst("java.util.List<", "").replace(">", "");
        String rostype = containedType;
        if (!isPrimitiveType(containedType)) {
          containedType = "edu.tufts.hrilab.rosade.msg." + containedType.replace("org.ros.message.", "");
        }
         builder.append(String.format("    %s %s = new %s();\n", field.getJavaTypeName(), list, field.getJavaTypeName().replaceFirst("List", "ArrayList")));
         builder.append(String.format("    for (%s e : val.%s()) {\n", determineType(field).replaceFirst("java.util.List<", "").replace(">", ""), getter));

        if (isPrimitiveType(containedType)) {
          builder.append(String.format("      %s.add(e); \n", list));
        } else {
          builder.append(String.format("      %s.add(%s.toRos(e, node));\n", list, containedType, getter));
        }

        builder.append(String.format("    }\n"));
        builder.append(String.format("    msg.%s(%s);\n", setter, list)); // Commented out to fix auto-gen code
      } else {
        String typeName = field.getJavaTypeName();
        // remove ros package structure from Time and Duration since we re-implement them
        if (typeName.contains("Time") || type.contains("Duration")) {
          typeName = typeName.replace("edu.tufts.hrilab.rosade.msg.", "org.ros.message.");
        }
        builder.append(String.format("    msg.%s(%s.toRos(val.%s(), node));\n", setter, javaType, getter, getter));
      }
    }
    //builder.append(String.format("    _pub.shutdown();"));
    builder.append(String.format("    return msg;\n  }\n"));

  }

  /**
   * Constructs the method which convets an ADE message to its ROS equivalent.
   *
   * @param messageContext the parsed representation of a ROS message
   * @param builder the file being generated
   */
  private void appendRosToAdeConverter(MessageContext messageContext, StringBuilder builder) {
    String instance = "_" + getInterfaceName().toLowerCase();
    String type = getPackageName() + "." + getInterfaceName();

    builder.append(String.format("  public static edu.tufts.hrilab.rosade.msg.%s.%s toAde(%s.%s val) {\n", getPackageName(), getInterfaceName(), getPackageName(), getInterfaceName()));
    builder.append(String.format("    edu.tufts.hrilab.rosade.msg.%s %s = new edu.tufts.hrilab.rosade.msg.%s();\n", type, instance, type));
    MessageFields fields = new MessageFields(messageContext);
    for (Field field : fields.getFields()) {
      if (field.isConstant()) {
        continue;
      }

      // Headers are special
      if (getInterfaceName().equals("Header") && field.getName().equals("stamp")) {
        continue;
      }

      String setter = messageContext.getFieldSetterName(field.getName());
      String getter = messageContext.getFieldGetterName(field.getName());
      String javaType = determineType(field);
      if (isPrimitiveType(determineType(field))) {
        builder.append(String.format("    %s.%s(val.%s());\n", instance, setter, getter));
      } else if (isContainer(field)) {
        String list = "cpy_" + field.getName().toLowerCase() + "List";
        String containedType = field.getJavaTypeName().replaceFirst("java.util.List<", "").replace(">", "");
        String rostype = containedType;
        if (!isPrimitiveType(containedType)) {
          containedType = "edu.tufts.hrilab.rosade.msg." + containedType.replace("org.ros.message.", "");
        }

        builder.append(String.format("    %s %s = new %s();\n", javaType, list, javaType.replaceFirst("List", "ArrayList")));
        builder.append(String.format("    for (%s e : val.%s()) {\n", rostype, getter));

        if (isPrimitiveType(containedType)) {
          builder.append(String.format("      %s.add(e);\n", list));
        } else {
          builder.append(String.format("      %s.add(%s.toAde(e));\n", list, containedType));
        }

        builder.append(String.format("    }\n"));
        builder.append(String.format("    %s.%s(%s);\n", instance, setter, list));
      } else {
        String typeName = field.getJavaTypeName();
        // remove ros package structure from Time and Duration since we re-implement them
        if (typeName.contains("Time") || type.contains("Duration")) {
          typeName = typeName.replace("org.ros.message.", "");
        }
        builder.append(String.format("    %s.%s(%s.toAde(val.%s()));\n", instance, setter, javaType, getter));
      }
    }
    builder.append(String.format("    return %s;\n  }\n", instance));
  }

  /**
   * Constructs the method which converts an ADE message to its ROS equivalent
   * by filling in passed in ADE message (instead of returning a new one).
   *
   * @param messageContext the parsed representation of a ROS message
   * @param builder the file being generated
   */
  private void appendRosToAdeCopyConverter(MessageContext messageContext, StringBuilder builder) {
    String instance = "_" + getInterfaceName().toLowerCase();
    String type = getPackageName() + "." + getInterfaceName();

    builder.append(String.format("  public static void toAde(%s.%s val, edu.tufts.hrilab.rosade.msg.%s %s) {\n", getPackageName(), getInterfaceName(), type, instance));
    MessageFields fields = new MessageFields(messageContext);
    for (Field field : fields.getFields()) {
      if (field.isConstant()) {
        continue;
      }

      // Headers are special
      if (getInterfaceName().equals("Header") && field.getName().equals("stamp")) {
        continue;
      }

      String setter = messageContext.getFieldSetterName(field.getName());
      String getter = messageContext.getFieldGetterName(field.getName());
      String javaType = determineType(field);
      if (isPrimitiveType(determineType(field))) {
        builder.append(String.format("    %s.%s(val.%s());\n", instance, setter, getter));
      } else if (isContainer(field)) {
        String list = "cpy_" + field.getName().toLowerCase() + "List";
        String containedType = field.getJavaTypeName().replaceFirst("java.util.List<", "").replace(">", "");
        String rostype = containedType;
        if (!isPrimitiveType(containedType)) {
          containedType = "edu.tufts.hrilab.rosade.msg." + containedType.replace("org.ros.message.", "");
        }

        builder.append(String.format("    %s %s = new %s();\n", javaType, list, javaType.replaceFirst("List", "ArrayList")));
        builder.append(String.format("    for (%s e : val.%s()) {\n", rostype, getter));

        if (isPrimitiveType(containedType)) {
          builder.append(String.format("      %s.add(e);\n", list));
        } else {
          builder.append(String.format("      %s.add(%s.toAde(e));\n", list, containedType));
        }

        builder.append(String.format("    }\n"));
        builder.append(String.format("    %s.%s(%s);\n", instance, setter, list));
      } else {
        String typeName = field.getJavaTypeName();
        // remove ros package structure from Time and Duration since we re-implement them
        if (typeName.contains("Time") || type.contains("Duration")) {
          typeName = typeName.replace("org.ros.message.", "");
        }
        builder.append(String.format("    %s.%s(%s.toAde(val.%s()));\n", instance, setter, javaType, getter));
      }
    }
    builder.append(String.format("  }\n"));
  }

  private void appendSettersAndGetters(MessageContext messageContext, StringBuilder builder) {
    MessageFields messageFields = new MessageFields(messageContext);
    Set<String> getters = Sets.newHashSet();
    for (Field field : messageFields.getFields()) {
      if (field.isConstant()) {
        continue;
      }

      String getter = messageContext.getFieldGetterName(field.getName());
      String setter = messageContext.getFieldSetterName(field.getName());
      if (getters.contains(getter)) {
                // In the case that two or more message fields have the same name except
        // for capitalization, we only generate a getter and setter pair for the
        // first one. The following fields will only be accessible via the
        // RawMessage interface.
        continue;
      }
      getters.add(getter);
      // getters
      builder.append(String.format("  public %s %s() {\n    return __%s; \n  }\n\n", determineType(field), getter,
              determineName(field)));
      // setters
      builder.append(String.format("  public void %s(%s value){\n    this.__%s = value;\n  }\n\n",
              setter, determineType(field), determineName(field)));
    }
  }

  private String determineType(Field field) {
    String type = field.getJavaTypeName();

    // headers are special
    if (getInterfaceName().equals("header") && type.equals("java.lang.String")) {
      return "edu.tufts.hrilab.rosade.msg.std_msgs.String";
    }

    // full type name
    if (!isPrimitiveType(field.getJavaTypeName()) && !isContainer(field)) {
      type = "edu.tufts.hrilab.rosade.msg." + type;
    }

    // remove package name for Time & Duration
    type = type.replace("org.ros.message.", "");

    // embed type info into collections
    int f = type.indexOf("<") + 1, b = type.indexOf(">");
    if (f != -1 && b != -1) {
      String sub = type.substring(f, b);
      if (!isPrimitiveType(sub)) {
        type = type.substring(0, f) + ""
                + ""
                + "edu.tufts.hrilab.rosade.msg." + sub + ">";
      }
    } else {
      type = type.replace("<", "<edu.tufts.hrilab.rosade.msg.");
    }
//        System.out.println(type);
    return type;
  }

  private String determineName(Field field) {
    String name = field.getName();
    if (name.equals("int") || name.equals("float") || name.equals("default")) {
      return name.substring(0, 1);
    } else {
      return name;
    }

  }

  private boolean isUnsupported(Field field) {
    String type = field.getJavaTypeName();
    return type.equals("std_msgs.Header");
  }

  private boolean isContainer(Field field) {
    return field.getJavaTypeName().contains("[]") || field.getJavaTypeName().contains("java.util.List");
  }

  private String getJavaValue(PrimitiveFieldType primitiveFieldType, String value) {
    switch (primitiveFieldType) {
      case BOOL:
        return Boolean.valueOf(!value.equals("0") && !value.equals("false")).toString();
      case FLOAT32:
        return value + "f";
      case STRING:
        return "\"" + escapeJava(value) + "\"";
      case BYTE:
      case CHAR:
      case INT8:
      case UINT8:
      case INT16:
      case UINT16:
      case INT32:
      case UINT32:
      case INT64:
      case UINT64:
      case FLOAT64:
        return value;
      default:
        throw new RosRuntimeException("Unsupported PrimitiveFieldType: " + primitiveFieldType);
    }
  }

  /**
   * Is this a primitive rosjava type? Notice that these are not necessarily
   * primitive Java types but the base types that rosjava supports.
   *
   * @param type the type in question
   * @return true i.f.f. primitive rosjava type
   */
  private boolean isPrimitiveType(String type) {
//        String type = field.getJavaTypeName();
    boolean primitive
            = type.equals("org.jboss.netty.buffer.ChannelBuffer")
            || type.equals("java.lang.String")
            || type.equals("boolean") || type.equals("boolean[]")
            || type.equals("int") || type.equals("int[]")
            || type.equals("double") || type.equals("double[]")
            || type.equals("float") || type.equals("float[]")
            || type.equals("byte") || type.equals("byte[]")
            || type.equals("char") || type.equals("char[]")
            || type.equals("long") || type.equals("long[]")
            || type.equals("short") || type.equals("short[]");

//        System.out.printf("%s: %s\n", type, primitive);
    return primitive;
  }

  private boolean hasOnlyConstants(MessageFields messageFields) {
    for (Field field : messageFields.getFields()) {
      if (isUnsupported(field)) {
        continue;
      }
      if (!field.isConstant()) {
        return false;
      }
    }
    return true;
  }
}
