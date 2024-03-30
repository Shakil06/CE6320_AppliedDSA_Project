#include <iostream>
#include <fstream>
#include <cctype>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <set>
#include <cstdlib> 
#include <algorithm>
#include <utility> 
#define INF 0x7FF00000


//Initial Graph after pharsing
class Graph {
protected:
    std::unordered_map<std::string, std::vector<std::string>> adjacencyList;

public:
    Graph() = default;

    void addEdge(std::string source, std::string destination) {
        adjacencyList[source].push_back(destination);
    }

    //Function to get the graph for weighted version
    std::unordered_map<std::string, std::vector<std::string>> getAdjacencyList() const {
        return adjacencyList;
    }
};


//Actual Weighted Graph to work with considering the final edge wight 0
class WeightedGraph : public Graph {
private:
    std::unordered_map<std::string, std::vector<std::pair<std::string, int>>> weightedAdjacencyList;

public:
    //For converting initial graph to weighted graph
    WeightedGraph(const Graph& graph) {
        std::unordered_map<std::string, std::vector<std::string>> adjList = graph.getAdjacencyList();
        std::unordered_map<std::string, int> sizes;

        for (const auto& pair : adjList) {
            std::string dest = pair.first;
            sizes[dest] = adjList[dest].size();
        }   

        for (const auto& pair : adjList) {
            std::string source = pair.first;
            for (std::string dest : pair.second) {
                int weight = sizes[dest]; //Adding fanout as weight considering the final edge wight 0 (output fanout 0)
                weightedAdjacencyList[source].push_back(std::make_pair(dest, weight)); 
            }
        }
    }

    //To get all vertices uniquely
    std::vector<std::string> getAllVertices() const {
        std::vector<std::string> allVertices;

        for (const auto& pair : weightedAdjacencyList) {
            allVertices.push_back(pair.first);  

            for (const auto& dest : pair.second) {
                allVertices.push_back(dest.first); 
            }
        }

        std::set<std::string> uniqueVertices(allVertices.begin(), allVertices.end());
        allVertices.assign(uniqueVertices.begin(), uniqueVertices.end());

        return allVertices;
    }

    //Function to find the shortest path between a source and destination node
    void dijkstraShortestPath(std::string source, std::string destination) {
        std::unordered_map<std::string, int> distance;
        std::unordered_map<std::string, std::string> previous;

        
        std::vector<std::string> allVertices = getAllVertices(); //To make distance-previous table

        //Initialize all distances to infinity and previous of the source node to null
        for (std::string vertex : allVertices) {
            distance[vertex] = INF;
            previous[vertex] = "null";
        }

        distance[source] = 0;  

        //Priority queue to update the distance-previous table
        std::priority_queue<std::pair<int, std::string>, std::vector<std::pair<int, std::string>>, std::greater<std::pair<int, std::string>>> pq;
        pq.push(std::make_pair(0, source));

        //Dijkstra's algorithm to compare and update the distance-previous table
        while (!pq.empty()) {
            std::string current = pq.top().second;
            pq.pop();

            for (const auto& edge : weightedAdjacencyList[current]) {
                std::string next = edge.first;
                int weight = edge.second;

                int totalDistance = distance[current] + weight;
                if (totalDistance < distance[next]) {
                    distance[next] = totalDistance;
                    previous[next] = current;
                    pq.push(std::make_pair(totalDistance, next));
                }
            }
        }

        //Reconstructing the path and reversing the list to get the path in order
        std::vector<std::string> shortestPath;
        for (std::string node = destination; node != "null"; node = previous[node]) {
            shortestPath.push_back(node);
        }
        std::reverse(shortestPath.begin(), shortestPath.end());

        //Printing the shortest path
        if (shortestPath.size() <= 1) {
            std::cout << "No path found from " << source << " to " << destination << std::endl;
        } else {
            std::cout << "Shortest path from " << source << " to " << destination << ": ";
            for (size_t i = 0; i < shortestPath.size(); ++i) {
                std::cout << shortestPath[i];
                if (i < shortestPath.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << " = " << distance[destination] <<std::endl; //weight
        }
    }

};


//Function to get gate nodes from a line
std::vector<std::string> extractStrings(const std::string& line) {
    std::vector<std::string> extractedStrings;
    std::string currentString;
    bool capturingString = false;

    for (char ch : line) {
            if (std::isalnum(ch) || ch == '_') {
                currentString += ch;
                capturingString = true;
            } else {
                if (capturingString && (ch == ',' || ch == ')' || std::isspace(ch))) {
                    if (!currentString.empty() && currentString[0] == 'G') {
                        extractedStrings.push_back(currentString);
                    }
                    currentString.clear();
                    capturingString = false;
                } else {
                    currentString.clear();
                    capturingString = false;
                }
            }
        }

        //Check for any remaining string at the end of the line
        if (capturingString) {
            if (!currentString.empty() && currentString[0] == 'G') {
                extractedStrings.push_back(currentString);
            }
        }

    return extractedStrings;
}


//Function to read the file, verify input-output and make initial graph
Graph findIntegersInLines(const std::string& fileName, const std::string &input, const std::string &output) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Wrong file name" << std::endl;
        exit(EXIT_FAILURE);
    }

    Graph graph;

    //Flag for correct input and output
    bool inputFound = false;
    bool outputFound = false;
    bool reverseFound = false;

    std::string line;
    
    while (std::getline(file, line)) {
        //Finding input and output in the benchfile
        if (line.find("INPUT(" + input + ")") == 0) {
            inputFound = true;
        } else if (line.find("OUTPUT(" + output + ")") == 0) {
            outputFound = true;
        }else if (line.find("OUTPUT(" + input + ")") == 0) {
            reverseFound = true;
        }

        //Getting the nodes from gate operation
        if (!line.empty() && std::toupper(line[0]) == 'G') {
            std::vector<std::string> nodes = extractStrings(line);

            std::string destinationNode = nodes[0]; //First integer is the destination node
            for (size_t i = 1; i < nodes.size(); ++i) {
                std::string sourceNode = nodes[i]; //Later integers are the source nodes
                graph.addEdge(sourceNode, destinationNode);
            }
        }
    }

    //Check for correct input and output
    if (!inputFound) {
        if (reverseFound) {
        std::cerr << "Signal " << input << " is not an input pin"<< std::endl;
        exit(EXIT_FAILURE);
        }
        std::cerr << "Signal " << input << " not found in file " << fileName << std::endl;
        exit(EXIT_FAILURE);
    }

    if (!outputFound) {
        std::cerr << "Signal " << output << " not found in file " << fileName << std::endl;
        exit(EXIT_FAILURE);
    }

    file.close();

    return graph;
}


int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Incorrect number of arguments" << std::endl;
        return 1;
    }

    std::string fileName = argv[1];
    std::string sourceNode = argv[2]; 
    std::string destinationNode = argv[3];
    
    Graph graph = findIntegersInLines(fileName, sourceNode, destinationNode);

    WeightedGraph weightedGraph(graph);

    weightedGraph.dijkstraShortestPath(sourceNode, destinationNode);

    return 0;
}