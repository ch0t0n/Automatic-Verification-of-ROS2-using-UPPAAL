import ast
import xml.etree.ElementTree as ET

enqueue_and_dequeue_func_uppaal = """
void enqueue(int &A[5], int n, int &cs, int &os, int p) {
    if (cs == n){
        int i;        
        for (i=0; i<=n-2; i++){
            A[i] = A[i+1];
        }
        A[n-1] = p;
        os = 1-os;
    }
    else {
        A[cs] = p;
        cs++;
    }
}

void dequeue(int &A[5], int n, int &cs) {    
    int i;
    if (cs == 1){
        A[0] = 0;
    }
    if (cs > 1) {
        for (i=0; i<=cs-2; i++){
            A[i] = A[i+1];
        }
        A[cs-1] = 0;
    }
    cs--;
    if (cs < 0){
        cs = 0;
    }
}
"""


def Add_global_variables(root, ros_model):
    # Setting up the global variables

    # Write the channels for the publishers and subscribers
    global_var = '\nchan '
    for pub in ros_model['Pub']:
        global_var += pub + ', '
    for sub in ros_model['Sub']:
        global_var += sub + ', '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Writing the rates of the timers
    global_var += 'const int '
    for k, v in ros_model['Timer'].items():
        global_var += k + ' = ' + str(v) + ', '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Writing the processing delay of the subscribers
    global_var += 'const int '
    for k, v in ros_model['D'].items():
        global_var += 'd'+k+' = '+str(v)+', '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Writing the queue sizes of the subscribers
    global_var += 'const int '
    for k, v in ros_model['Qsize'].items():
        global_var += 'n'+k+' = '+str(v)+', '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Finding the maximum Qsize
    qsizes = []
    for k, v in ros_model['Qsize'].items():
        qsizes.append(v)
    max_size = max(qsizes)

    # Writing the queues for each subscriber
    global_var += 'int '
    for sub in ros_model['Sub']:
        global_var += 'A'+sub+'['+str(max_size)+'], '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Writing the variables for overflow and current size for each queue
    global_var += 'int '
    for sub in ros_model['Sub']:
        global_var += 'c'+sub+' = 0, ' + 'o'+sub+' = 0, '
    global_var = global_var[:-2] # removing the last comma
    global_var += ';\n'

    # Writing the enqueue and dequeue functions for UPPAAAL
    global_var += enqueue_and_dequeue_func_uppaal

    # Adding the global declarations in the xml file
    global_declarations = ET.SubElement(root, "declaration")
    global_declarations.text = global_var


def Add_system_variables(root, ros_model):
    system = ET.SubElement(root, "system")
    sys_text = '\n'
    sys_def = 'system '

    # Adding the publisher automata
    for pub in ros_model['Pub']:
        sys_text += 'Process'+pub+' = '+pub+'name();\n'
        sys_def += 'Process'+pub+', '

    # Adding the subscriber automata
    for sub in ros_model['Sub']:
        sys_text += 'Process'+sub+' = '+sub+'name();\n'
        sys_def += 'Process'+sub+', '

    # Adding the topic automata
    for top in ros_model['Topic']:
        sys_text += 'Process'+top+' = '+top+'name();\n'
        sys_def += 'Process'+top+', '

    # Adding all system variables in the xml file
    system.text = sys_text + sys_def[:-2] +';\n'

def Add_network_of_TA(root, ros_model):    
    loc_i = 0 # Location id    
    # Adding the publisher automata
    for pub, rate in ros_model['MT'].items():
        pub_ta = ET.SubElement(root, "template")
        # Adding the name
        pub_name = ET.SubElement(pub_ta, "name")
        pub_name.text = pub+'name'
        # Adding a clock
        clock_dec = ET.SubElement(pub_ta, "declaration")
        clock_dec.text = 'clock t;'
        # Initial location and id
        id_val = 'id'+str(loc_i)
        init_id = id_val
        loc_dec = ET.SubElement(pub_ta, "location", id=id_val)
        loc_name = ET.SubElement(loc_dec, "name")
        loc_name.text = 'init'+pub
        
        if 'r' in rate: # if the publisher has timer
            # Add an invariant
            loc_inv = ET.SubElement(loc_dec, "label", kind="invariant")
            loc_inv.text = 't <= '+rate
            init_loc = ET.SubElement(pub_ta, "init", ref=init_id)            
            # Adding a transition for the location
            transition = ET.SubElement(pub_ta, "transition")
            s_element = ET.SubElement(transition, "source", ref=id_val)
            t_element = ET.SubElement(transition, "target", ref=id_val)
            guard = ET.SubElement(transition, "label", kind="guard")
            guard.text = 't == '+rate
            action = ET.SubElement(transition, "label", kind="synchronisation")
            action.text = pub+'!'
            reset_clock = ET.SubElement(transition, "label", kind="assignment")
            reset_clock.text = 't=0'
        else: # Else the publisher has subscriber            
            # Add another location
            loc_i2 = loc_i + 1
            id_val2 = 'id'+str(loc_i2)
            loc_received = ET.SubElement(pub_ta, "location", id=id_val2)
            loc_name = ET.SubElement(loc_received, "name")
            loc_name.text = 'received'
            # Add an invariant
            loc_inv = ET.SubElement(loc_received, "label", kind = "invariant")
            loc_inv.text = 't <= 0'
            init_loc = ET.SubElement(pub_ta, "init", ref=init_id)
            # Adding a transition from init to received
            trans_init2received = ET.SubElement(pub_ta, "transition")
            s_element = ET.SubElement(trans_init2received, "source", ref=id_val)
            t_element = ET.SubElement(trans_init2received, "target", ref=id_val2)
            action = ET.SubElement(trans_init2received, "label", kind="synchronisation")
            action.text = rate+'?'
            reset_clock = ET.SubElement(trans_init2received, "label", kind="assignment")
            reset_clock.text = 't=0'
            # Adding a transition from received to init
            trans_received2init = ET.SubElement(pub_ta, "transition")
            s_element = ET.SubElement(trans_received2init, "source", ref=id_val2)
            t_element = ET.SubElement(trans_received2init, "target", ref=id_val)
            action = ET.SubElement(trans_received2init, "label", kind="synchronisation")
            action.text = pub+'!'
            loc_i = loc_i2
        
        loc_i += 1
    
    # Adding the subscriber automata
    for sub, rate in ros_model['D'].items():
        sub_ta = ET.SubElement(root, "template")
        # Adding the name
        sub_name = ET.SubElement(sub_ta, "name")
        sub_name.text = sub+'name'
        # Adding a clock
        clock_dec = ET.SubElement(sub_ta, "declaration")
        clock_dec.text = 'clock t;'
        # Initial location and id
        id_val = 'id'+str(loc_i)
        loc_dec = ET.SubElement(sub_ta, "location", id=id_val)
        loc_name = ET.SubElement(loc_dec, "name")
        loc_name.text = 'init'+sub
        init_loc = ET.SubElement(sub_ta, "init", ref=id_val)
        # Add an invariant
        loc_inv = ET.SubElement(loc_dec, "label", kind="invariant")
        loc_inv.text = 't <= d'+sub
        # Adding a transition for the location
        trans_sub = ET.SubElement(sub_ta, "transition")
        s_element = ET.SubElement(trans_sub, "source", ref=id_val)
        t_element = ET.SubElement(trans_sub, "target", ref=id_val)
        guard = ET.SubElement(trans_sub, "label", kind="guard")
        guard.text = 't == d'+sub
        for k, v in ros_model['MT'].items():            
            if (v == sub):
                action = ET.SubElement(trans_sub, "label", kind="synchronisation")
                action.text = sub+'!'
                break
        reset_clock = ET.SubElement(trans_sub, "label", kind="assignment")
        reset_clock.text = 't=0, dequeue(A'+sub+',n'+sub+',c'+sub+')'
        loc_i += 1

    # Adding the topic automata
    for top in ros_model['Topic']:
        top_ta = ET.SubElement(root, "template")
        # Adding the name
        top_name = ET.SubElement(top_ta, "name")
        top_name.text = top+'name'

        # Initial location and id
        id_val = 'id'+str(loc_i)
        loc_dec = ET.SubElement(top_ta, "location", id=id_val)
        loc_name = ET.SubElement(loc_dec, "name")
        loc_name.text = 'init'+top
        init_loc = ET.SubElement(top_ta, "init", ref=id_val)

        # Get the list of publishers and subscribers
        pubt, subt = [], []
        for k,v in ros_model['PT'].items():
            if (v==top):
                pubt.append(k)
        for k,v in ros_model['ST'].items():
            if (v==top):
                subt.append(k)
        loc_i += 1
        
        # Add the transitions
        for pub in pubt:
            trans_top = ET.SubElement(top_ta, "transition")
            s_element = ET.SubElement(trans_top, "source", ref=id_val)
            t_element = ET.SubElement(trans_top, "target", ref=id_val)
            action = ET.SubElement(trans_top, "label", kind="synchronisation")
            action.text = pub+'?'
            pub_val = pub[-1]
            enqueue_text = ''
            for sub in subt:
                enqueue_text += 'enqueue(A'+sub+',n'+sub+',c'+sub+',o'+sub+','+pub[-1]+'),'
            enqueue_text = enqueue_text[:-1]
            enqueue_p = ET.SubElement(trans_top, "label", kind="assignment")
            enqueue_p.text = enqueue_text

def main():
    # Reading the ros model from a text file
    with open('example_ros_model.txt') as f:
        data = f.read()
    ros_model = ast.literal_eval(data)

    # Initializing the root of the xml file
    root = ET.Element("nta")

    # Setting up the global variables
    Add_global_variables(root, ros_model)    

    # Writing the xml file for the network of TA
    Add_network_of_TA(root, ros_model)

    # Setting up the system variables
    Add_system_variables(root, ros_model)

    # Writing the tree to the xml file
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ', level=0)
    # ET.dump(root) # For printing in std_output
    tree.write("example_ros_model.xml", encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    main()