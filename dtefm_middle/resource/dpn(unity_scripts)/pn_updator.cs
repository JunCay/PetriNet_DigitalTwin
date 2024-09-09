using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using PlaceMsg = RosMessageTypes.DtefmInterfaces.PlaceMsgMsg;
using TransitionMsg = RosMessageTypes.DtefmInterfaces.TransitionMsgMsg;
using ArcMsg = RosMessageTypes.DtefmInterfaces.ArcMsgMsg;
using PetriNegMsg = RosMessageTypes.DtefmInterfaces.PetriNetMsg;
using PNCommandRequest = RosMessageTypes.DtefmInterfaces.PNCommandRequest;
using PNCommandResponse = RosMessageTypes.DtefmInterfaces.PNCommandResponse;
using System.Linq;
using TMPro;
using UnityEngine;

public class pn_updator : MonoBehaviour
{
    public GameObject placePrefab;
    public GameObject transitionPrefab;
    public GameObject PN;
    public float initiate_position_y = -50;
    public MeshRenderer arcPrefab;
    public MeshRenderer arcPrefab_unready;
    public float arc_xscale;
    public float arc_yscale;

    public float areaSize;
    public float attractionForce;
    public float repulsionForce;
    public int iterations;
    public bool imortal = false;

    private GameObject currentPlace;
    private GameObject currentTransition;

    private ROSConnection ros;
    private PlaceMsg[] places;
    private TransitionMsg[] transitions;
    private ArcMsg[] arcs;

    private Dictionary<string, GameObject> node_dict = new Dictionary<string, GameObject>();
    private Dictionary<string, List<string>> adjacency_id = new Dictionary<string, List<string>>();
    private Dictionary<string, MeshRenderer> arc_arrow_dict = new Dictionary<string, MeshRenderer>();

    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.instance.Subscribe<PetriNegMsg>("identity/pn/update", PNUpdateCallback);
        arc_xscale = arcPrefab.transform.localScale.x;
        arc_yscale = arcPrefab.transform.localScale.y;
    }

    void PNUpdateCallback(PetriNegMsg msg)
    {
        places = msg.places;
        transitions = msg.transitions;
        arcs = msg.arcs;

        PNNodeUpdate();
        // PNShowAdj();
        // PNLogger();
        StartCoroutine(LayoutNodesCoroutine());
    }

    // Update is called once per frame
    void Update()
    {
        PNDrawArc();
    }

    void PNDrawArc()
    {
        foreach (var edge in adjacency_id)
        {
            string node_from_id = edge.Key;
            foreach (string node_to_id in edge.Value)
            {
                PNDrawArcSingle(node_from_id, node_to_id);
            }
        }
    }

    void PNDrawArcSingle(string node_from_id, string node_to_id)
    {
        MeshRenderer mr;
        MeshRenderer mr_;       // unready arrow
        if (node_dict.Keys.Contains(node_from_id) && node_dict.Keys.Contains(node_to_id))
        {
            string arc_arrow_key = node_from_id + node_to_id;
            if (arc_arrow_dict.Keys.Contains(arc_arrow_key))
            {
                mr = arc_arrow_dict[arc_arrow_key];
                mr_ = arc_arrow_dict[arc_arrow_key + "_"];
            }
            else
            {
                mr = Instantiate(arcPrefab);
                mr.transform.SetParent(PN.transform);
                mr.name = node_dict[node_from_id].name + "->" + node_dict[node_to_id].name;
                mr_ = Instantiate(arcPrefab_unready);
                mr_.transform.SetParent(PN.transform);
                mr_.name = node_dict[node_from_id].name + "->" + node_dict[node_to_id].name + "_";

                arc_arrow_dict[arc_arrow_key] = mr;
                arc_arrow_dict[arc_arrow_key + "_"] = mr_;

            }

            float length = Vector3.Distance(node_dict[node_from_id].transform.position, node_dict[node_to_id].transform.position);
            mr.transform.localScale = new Vector3(arc_xscale, length, 1);
            mr.transform.position = (node_dict[node_from_id].transform.position + node_dict[node_to_id].transform.position) / 2;
            mr.transform.LookAt(node_dict[node_to_id].transform.position);
            mr.transform.Rotate(90, 0, 0);
            mr.material.mainTextureScale = new Vector2(1, length * arc_yscale);

            mr_.transform.localScale = new Vector3(arc_xscale, length, 1);
            mr_.transform.position = (node_dict[node_from_id].transform.position + node_dict[node_to_id].transform.position) / 2;
            mr_.transform.LookAt(node_dict[node_to_id].transform.position);
            mr_.transform.Rotate(90, 0, 0);
            mr_.material.mainTextureScale = new Vector2(1, length * arc_yscale);

            if ( node_dict[node_from_id].GetComponent<transition_property>() )
            {
                if ( node_dict[node_from_id].GetComponent<transition_property>().status == "ready" )
                {
                    mr.gameObject.SetActive(true);
                    mr_.gameObject.SetActive(false);
                }
                else
                {
                    mr.gameObject.SetActive(false);
                    mr_.gameObject.SetActive(true);
                }
            
            }
            else if (node_dict[node_to_id].GetComponent<transition_property>())
            {
                if (node_dict[node_to_id].GetComponent<transition_property>().status == "ready")
                {
                    mr.gameObject.SetActive(true);
                    mr_.gameObject.SetActive(false);
                }
                else
                {
                    mr.gameObject.SetActive(false);
                    mr_.gameObject.SetActive(true);
                }
            }
        }
    }


    void PNNodeUpdate()
    {
        for (int i = 0; i < places.Length; i++)
        {
            if (!node_dict.ContainsKey(places[i].id))
            {
                Vector3 position = new Vector3(Random.Range(-areaSize, areaSize), initiate_position_y, Random.Range(-areaSize, areaSize));
                currentPlace = Instantiate(placePrefab, position, Quaternion.identity);
                currentPlace.transform.SetParent(PN.transform);
                node_dict[places[i].id] = currentPlace;
                TextMeshPro textComponent = currentPlace.GetComponentInChildren<TextMeshPro>();
                textComponent.text = places[i].name + "\ntokens: " + places[i].tokens;
                currentPlace.name = places[i].name;
                currentPlace.GetComponent<place_property>().id = places[i].id;
                currentPlace.GetComponent<place_property>().name_ = places[i].name;
                currentPlace.GetComponent<place_property>().ins = places[i].ins;
                currentPlace.GetComponent<place_property>().outs = places[i].outs;
                currentPlace.GetComponent<place_property>().in_arcs = places[i].in_arcs;
                currentPlace.GetComponent<place_property>().out_arcs = places[i].out_arcs;
                for (int j = 0; j < places[i].marking_types.Length; j++)
                {
                    currentPlace.GetComponent<place_property>().marking[places[i].marking_types[j]] = places[i].marking[j];
                }
                if (!adjacency_id.Keys.Contains(places[i].id))
                {
                    adjacency_id[places[i].id] = new List<string>();
                }
            }
            else
            {
                currentPlace = node_dict[places[i].id];
                TextMeshPro textComponent = currentPlace.GetComponentInChildren<TextMeshPro>();
                textComponent.text = places[i].name + "\ntokens: " + places[i].tokens;
                currentPlace.GetComponent<place_property>().id = places[i].id;
                currentPlace.GetComponent<place_property>().name_ = places[i].name;
                currentPlace.GetComponent<place_property>().ins = places[i].ins;
                currentPlace.GetComponent<place_property>().outs = places[i].outs;
                currentPlace.GetComponent<place_property>().in_arcs = places[i].in_arcs;
                currentPlace.GetComponent<place_property>().out_arcs = places[i].out_arcs;
                for (int j = 0; j < places[i].marking_types.Length; j++)
                {
                    currentPlace.GetComponent<place_property>().marking[places[i].marking_types[j]] = places[i].marking[j];
                }
                if (!adjacency_id.Keys.Contains(places[i].id))
                {
                    adjacency_id[places[i].id] = new List<string>();
                }
            }
        }
        for (int i = 0; i < transitions.Length; i++)
        {
            if (!node_dict.ContainsKey(transitions[i].id))
            {
                Vector3 position = new Vector3(Random.Range(-areaSize, areaSize), initiate_position_y, Random.Range(-areaSize, areaSize));
                currentTransition = Instantiate(transitionPrefab, position, Quaternion.identity);
                currentTransition.transform.SetParent(PN.transform);
                node_dict[transitions[i].id] = currentTransition;
                TextMeshPro textComponent = currentTransition.GetComponentInChildren<TextMeshPro>();
                textComponent.text = transitions[i].name + "\n" + transitions[i].status + "&" + transitions[i].work_status;
                currentTransition.name = transitions[i].name;
                currentTransition.GetComponent<transition_property>().id = transitions[i].id;
                currentTransition.GetComponent<transition_property>().name_ = transitions[i].name;
                currentTransition.GetComponent<transition_property>().ins = transitions[i].ins;
                currentTransition.GetComponent<transition_property>().outs = transitions[i].outs;
                currentTransition.GetComponent<transition_property>().in_arcs = transitions[i].in_arcs;
                currentTransition.GetComponent<transition_property>().out_arcs = transitions[i].out_arcs;
                currentTransition.GetComponent<transition_property>().time = transitions[i].time;
                currentTransition.GetComponent<transition_property>().priority = transitions[i].priority;
                currentTransition.GetComponent<transition_property>().status = transitions[i].status;
                currentTransition.GetComponent<transition_property>().work_status = transitions[i].work_status;
                UpdateTransitionColor(currentTransition);

                if (!adjacency_id.Keys.Contains(transitions[i].id))
                {
                    adjacency_id[transitions[i].id] = new List<string>();
                }
            }
            else 
            {
                currentTransition = node_dict[transitions[i].id];
                TextMeshPro textComponent = currentTransition.GetComponentInChildren<TextMeshPro>();
                textComponent.text = transitions[i].name + "\n" + transitions[i].status + "&" + transitions[i].work_status;
                currentTransition.GetComponent<transition_property>().id = transitions[i].id;
                currentTransition.GetComponent<transition_property>().name_ = transitions[i].name;
                currentTransition.GetComponent<transition_property>().ins = transitions[i].ins;
                currentTransition.GetComponent<transition_property>().outs = transitions[i].outs;
                currentTransition.GetComponent<transition_property>().in_arcs = transitions[i].in_arcs;
                currentTransition.GetComponent<transition_property>().out_arcs = transitions[i].out_arcs;
                currentTransition.GetComponent<transition_property>().time = transitions[i].time;
                currentTransition.GetComponent<transition_property>().priority = transitions[i].priority;
                currentTransition.GetComponent<transition_property>().status = transitions[i].status;
                currentTransition.GetComponent<transition_property>().work_status = transitions[i].work_status;
                UpdateTransitionColor(currentTransition);

            }
        }
        for (int i = 0; i < arcs.Length; i++)
        {
            if (!adjacency_id[arcs[i].node_in].Contains(arcs[i].node_out))
            {
                adjacency_id[arcs[i].node_in].Add(arcs[i].node_out);
            }
        }
    }

    void UpdateTransitionColor(GameObject gb)
    {
        Renderer transition_renderer = gb.GetComponent<Renderer>();
        if (gb.GetComponent<transition_property>().work_status == "firing")
        {
            transition_renderer.material.color = Color.blue;
        }
        else
        {
            if (gb.GetComponent<transition_property>().status == "ready")
            {
                transition_renderer.material.color = Color.green;

            }
            else
            {
                transition_renderer.material.color = Color.red;
            }
        }

    }

    void PNLogger()
    {
        List<string> place_names = new List<string>();
        List<string> transition_names = new List<string>();
        List<string> arc_names = new List<string>();

        for (int i = 0; i < places.Length; i++)
        {
            place_names.Add(places[i].name);
        }
        for (int i = 0; i < transitions.Length; i++)
        {
            transition_names.Add(transitions[i].name);
        }
        for (int i = 0; i < arcs.Length; i++)
        {
            arc_names.Add(arcs[i].name);
        }
        Debug.Log(string.Join(", ", place_names));
        Debug.Log(string.Join(", ", transition_names));
        Debug.Log(string.Join(", ", arc_names));
    }

    void PNShowAdj()
    {
        int scale = node_dict.Count;
        List<string> keys = node_dict.Keys.ToList();

        string[,] adjMatrix = new string[scale + 1, scale + 1];
        adjMatrix[0, 0] = "\\";
        for (int i = 1; i < scale + 1; i++)
        {
            adjMatrix[i, 0] = node_dict[keys[i - 1]].name;
            adjMatrix[0, i] = node_dict[keys[i - 1]].name;
        }
        for (int i = 1; i < scale + 1; i++)
        {
            for (int j = 1; j < scale + 1; j++)
            {
                if (adjacency_id[keys[i - 1]].Contains(keys[j - 1]))
                {
                    adjMatrix[i, j] = "1";
                }
                else
                {
                    adjMatrix[i, j] = "0";
                }

            }
        }
        string matrixString = "adjMatrix: \n";
        for (int i = 0; i < scale + 1; i++)
        {
            for (int j = 0; j < scale + 1; j++)
            {
                matrixString += adjMatrix[i, j] + "\t";
            }
            matrixString += "\n"; // 用换行符分隔行
        }

        Debug.Log(matrixString);

    }

    IEnumerator LayoutNodesCoroutine()
    {
        int scale = node_dict.Count;
        List<string> keys = node_dict.Keys.ToList();

        for (int iter = 0; iter < iterations;)
        {
            // repulsive force between nodes
            for (int i = 0; i < scale; i++)
            {
                Vector3 displacement = Vector3.zero;
                for (int j = 0; j < scale; j++)
                {
                    if (i != j)
                    {
                        Vector3 direction = node_dict[keys[i]].transform.position - node_dict[keys[j]].transform.position;
                        float distance = direction.magnitude;
                        if (distance > 0)
                        {
                            displacement += (direction.normalized / distance) * repulsionForce;
                        }
                    }
                }
                node_dict[keys[i]].transform.position += displacement * Time.deltaTime;
            }

            // attractive force between nodes
            foreach (var edge in adjacency_id)
            {
                string i = edge.Key;
                foreach (string j in edge.Value)
                {
                    Vector3 direction = node_dict[j].transform.position - node_dict[i].transform.position;
                    float distance = direction.magnitude;
                    if (distance > 0)
                    {
                        Vector3 attraction = direction.normalized * Mathf.Log(distance) * attractionForce;
                        node_dict[i].transform.position += attraction * Time.deltaTime;
                        node_dict[j].transform.position -= attraction * Time.deltaTime;
                    }
                }
            }
            if(!imortal)
            {
                iter++;
            }
            yield return null;
        }
    }
}
