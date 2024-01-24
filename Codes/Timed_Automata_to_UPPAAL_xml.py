import ast
import xml.etree.ElementTree as ET

loc_i = 0  # Global reference counter for each location

# Timed_Automata = {
#     'L': {'far','near','in'},
#     'l0': 'far',
#     'Act':{'exi!','approach!'},
#     'g': {'E1':None, 'E2':None, 'E3':'y>2'},
#     'C':{'y'},
#     'I':{'far':None,'near':'y<=5','in':'y<=5'},
#     'E':{'E1': ('far',None,'approach!',{'y'},'near'),
#          'E2': ('in',None,'exi!',None,'far'),
#          'E3': ('near','y>2',None,None,'in')}
# }

def Add_automaton_to_root(Timed_Automaton, TA_name, root):
    # Initializing the Automation
    automaton = ET.SubElement(root, "template")
    auto_name = ET.SubElement(automaton, "name")
    auto_name.text = TA_name

    # Getting the clocks for the Automaton
    clocks_dec = ET.SubElement(automaton, "declaration")
    clock_var = ''
    for clock in Timed_Automaton['C']:
        clock_var += 'clock ' + clock + ';'
    clocks_dec.text = clock_var

    # Getting the locations and their invariants
    for i, loc in enumerate(Timed_Automaton['I']):
        global loc_i
        id_val = 'id' + str(loc_i)
        loc_element = ET.SubElement(automaton, "location", id=id_val)
        loc_name = ET.SubElement(loc_element, "name")
        loc_name.text = loc
        if Timed_Automaton['I'][loc]:  # Checking if there are any invariants
            loc_inv = ET.SubElement(loc_element, "label", kind="invariant")
            loc_inv.text = Timed_Automaton['I'][loc]
        loc_i += 1

    # Getting the references of each location
    refs = []
    for loc in automaton.findall('location'):
        loc_ref = loc.attrib['id']
        loc_name = loc.find('name').text
        refs.append((loc_ref, loc_name))
        if loc_name == Timed_Automaton['l0']:
            init = ET.SubElement(automaton, "init", ref=loc_ref)  # Getting the initial location

    # Getting the transitions for the timed automaton
    for key, edge in Timed_Automaton['E'].items():
        transition = ET.SubElement(automaton, "transition")
        source, target = '', ''
        for v in refs:
            if edge[0] == v[1]:
                source = v[0]
            elif edge[4] == v[1]:
                target = v[0]
        s_element = ET.SubElement(transition, "source", ref=source)
        t_element = ET.SubElement(transition, "target", ref=target)
        if edge[1]:  # Guard condition present
            guard = ET.SubElement(transition, "label", kind="guard")
            guard.text = edge[1]
        if edge[2]:  # Action is present
            if ('!' in edge[2]) or ('?' in edge[2]):
                action = ET.SubElement(transition, "label", kind="synchronisation")
                action.text = edge[2]
        if edge[3]:  # Reset clock is present
            reset_clock = ET.SubElement(transition, "label", kind="assignment")
            cs = ''
            for c in edge[3]:
                cs += c + ','
            cs = cs[:-1]
            cs += ':=0'
            reset_clock.text = cs


def Add_global_variables(root, network_of_TA):
    # Setting up the global variables
    global_var = '\n'
    for key, Timed_Automaton in network_of_TA.items():
        for act in Timed_Automaton['Act']:
            if '!' in act:
                global_var += 'chan ' + act[:-1] + ';\n'
    # print(global_var)
    global_declarations = ET.SubElement(root, "declaration")
    global_declarations.text = global_var


def Add_system_variables(root, network_of_TA):
    # Adding the system variables
    system = ET.SubElement(root, "system")
    sys_text = '\n'
    for key, value in network_of_TA.items():
        sys_text += 'Process' + str(key) + ' = ' + str(key) + '();\n'
    sys_text += 'system '
    for key, value in network_of_TA.items():
        sys_text += 'Process' + str(key) + ','
    sys_text = sys_text[:-1]
    sys_text += ';'
    system.text = sys_text


def main():
    # Reading network of TA from a text file
    with open('ta_input.txt') as f:
        data = f.read()
    network_of_TA = ast.literal_eval(data)

    # Initializing the root of the xml file
    root = ET.Element("nta")

    Add_global_variables(root, network_of_TA)  # Adding the global variables
    for name, Timed_Automaton in network_of_TA.items():  # Adding each TA in the model
        Add_automaton_to_root(Timed_Automaton, name, root)
    Add_system_variables(root, network_of_TA)  # Adding the system variables

    # Writing the tree to the xml file
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ', level=0)
    # ET.dump(root) # For printing in std_output
    tree.write("timed_automata_for_uppaal.xml", encoding='utf-8', xml_declaration=True)


if __name__ == "__main__":
    main()
