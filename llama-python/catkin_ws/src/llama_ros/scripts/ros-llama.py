#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from llama_cpp import Llama

input_language = "ro"  # Romanian

# Load action and object vocab

if input_language == "en":
    action_file = "./root/json/en/actions.json"
    object_file = "/root/json/en/objects.json"
elif input_language == "ro":
    action_file = "/root/json/ro/actiuni.json"
    object_file = "/root/json/ro/obiecte.json"


with open(action_file) as f:
    action_map = json.load(f)
    valid_actions = list(action_map.keys())

with open(object_file) as f:
    object_map = json.load(f)
    valid_objects = list(object_map.values())

# Make sure the lists are initialized
if not valid_actions:
    rospy.logerr("No valid actions found in actions.json")
    valid_actions = ["default_action"]
    exit(1)
if not valid_objects:
    rospy.logerr("No valid objects found in objects.json")
    valid_objects = ["default_object"]
    exit(1)

# Initialize the model
llm = Llama(
    model_path="/root/models/llama3.2/unsloth/Llama-3.2-3B-Instruct-Q8_0.gguf",
    n_ctx=1024,
    n_threads=16,
)

def make_command_prompt(user_input):
    return f"""<|start_header_id|>system<|end_header_id|>
You are a command interpreter that converts natural language into structured action commands.

Respond with a flat Python-style list like:
['action1', 'object1', 'action2', 'object2', ...]
or ['action1', 'object1', 'object2', 'action2', 'object3']

🧠 Use the following valid actions:
{valid_actions}

📦 Use the following valid object names:
{valid_objects}

If the input includes multiple steps, break them down in order.
Write only the command list, no additional text, no emojis, no explanations.
Give only the array, no other text, no errors or invalid objects.
Keep in mind that there are sentences that may use "it" or "them" to refer to previous objects, use the context to resolve these and put the correct object names in the list.
Input: {user_input}
Output:
<|eot_id|><|start_header_id|>assistant<|end_header_id|>
"""

def input_callback(msg):
    user_input = msg.data.strip()
    rospy.loginfo(f"Received input: {user_input}")

    command_prompt = make_command_prompt(user_input)

    assistant_response = ""
    try:
        for chunk in llm(
            command_prompt,
            max_tokens=200,
            stop=["<|eot_id|>"],
            temperature=0.2,
            top_p=0.95,
            repeat_penalty=1.1,
            stream=True
        ):
            token = chunk["choices"][0]["text"]
            assistant_response += token

        rospy.loginfo(f"Parsed commands: {assistant_response}")
        pub.publish(assistant_response)

    except Exception as e:
        rospy.logerr(f"Error during LLaMA inference: {e}")

if __name__ == '__main__':
    rospy.init_node('llama_parser_node')
    pub = rospy.Publisher('/llama_parser/commands', String, queue_size=10)
    sub = rospy.Subscriber('/llama_parser/input', String, input_callback)
    rospy.loginfo("LLaMA parser node started, waiting for input...")
    rospy.spin()
