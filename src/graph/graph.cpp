#include "graph.h"


Graph::Vertex::Vertex(int id)
{
    _id = id;
    _position = Vector2f(0.0, 0.0);
}


Graph::Vertex::Vertex(int id, Vector2f position)
{
    _id = id;
    _position = position;
}

Graph::Vertex::~Vertex()
{

}

Graph::EdgeSet Graph::Vertex::enteringEdges()
{
    Graph::EdgeSet eset;
    for (EdgeSet::const_iterator e = _edges.begin(); e != _edges.end(); e++) {
        
		if (((*e)->to() == this) || ((*e)->undirected()))
			eset.insert(*e);
	}

	return eset;
}

Graph::EdgeSet Graph::Vertex::exitingEdges()
{
    Graph::EdgeSet eset;
    for (EdgeSet::const_iterator e = _edges.begin(); e != _edges.end(); e++) {
        
		if (((*e)->from() == this) || ((*e)->undirected()))
			eset.insert(*e);
	}

	return eset;
}



Graph::Vertex* Graph::addVertex(const int &id, Vector2f position)
{
    Vertex* v = new Graph::Vertex(id, position);

    bool result = Graph::addVertex(v);

    if (result){
        return v;
    }
    else {
        delete v;
        return 0;
    }
}

bool Graph::addVertex(Vertex* v)
{
    auto result = _vertices.insert(std::make_pair(v->id(), v));

    if (!result.second){
        return false;
    }
    
    _verticesIdCount++;

    return true;
}


Graph::Edge::Edge(Vertex* from, Vertex* to, bool undirected, int id, float cost, int capacity)
{   
	_from = from;
	_to = to;
    _id = id;


	_undirected = undirected;
	_cost = cost;
	_capacity = capacity;
    _parentId = id;

    _vertices.insert(from);
    _vertices.insert(to);
    
}

Graph::Edge::~Edge()
{

}


bool Graph::Edge::operator<(const Edge& other) const
{
    return _id < other.id();
}


Graph::Edge* Graph::addEdge(Vertex* from, Vertex* to, bool undirected, float cost, int capacity)
{   
    if (cost < 0){
        Vector2f vertexDifference = to->position() - from->position();
        cost = vertexDifference.norm();
    }

    int id = _edgesIdCount + 1;

    Edge* e = new Graph::Edge(from, to, undirected, id, cost, capacity);
    
    bool result = Graph::addEdge(e);

    if (result){
        return e;
    }
    else{
        delete e;
        return 0;
    }


}

Graph::Edge* Graph::addEdge(int fromId, int toId, bool undirected, float cost, int capacity)
{   
    Vertex* from = vertex(fromId);
    Vertex* to = vertex(toId);

    if (from == 0 || to == 0){
        return 0;
    }

    if (cost < 0){
        Vector2f vertexDifference = to->position() - from->position();
        cost = vertexDifference.norm();
    }


    int id = _edgesIdCount + 1;

    Edge* e = new Graph::Edge(from, to, undirected, id, cost, capacity);
    
    bool result = Graph::addEdge(e);

    if (result){
        return e;
    }
    else{
        delete e;
        return 0;
    }

}




bool Graph::addEdge(Edge* e)
{
    auto result = _edges.insert(std::make_pair(e->id(), e));

    if (!result.second){
        return false;
    }

    _edgesIdCount++;

    for (Vertex* v : e->vertices()){
        if (v){
            v->edges().insert(e);
        }
    }

    return true;

}



Graph::Graph()
{
    _verticesIdCount = 0;
    _edgesIdCount = 0;
}


Graph::Vertex* Graph::vertex(int id)
{
    VertexIDMap::iterator it =_vertices.find(id);
    if (it == _vertices.end()){
        return nullptr;
    }
    return it->second;
}

const Graph::Vertex* Graph::vertex(int id) const
{
    VertexIDMap::const_iterator it =_vertices.find(id);
    if (it == _vertices.end()){
        return nullptr;
    }
    return it->second;
}

Graph::Edge* Graph::edge(int id)
{
    EdgeIDMap::iterator it =_edges.find(id);
    if (it == _edges.end()){
        return nullptr;
    }
    return it->second;
}

const Graph::Edge* Graph::edge(int id) const
{
    EdgeIDMap::const_iterator it =_edges.find(id);
    if (it == _edges.end()){
        return nullptr;
    }
    return it->second;
}



Graph::~Graph()
{
    clear();
}

void Graph::clear()
{
    for (VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it)
        delete (it->second);
    for (EdgeIDMap::iterator it=_edges.begin(); it!=_edges.end(); ++it)
        delete (it->second);

    _vertices.clear();
    _edges.clear();

    _verticesIdCount = 0;
    _edgesIdCount = 0;
}

Graph::Graph( const Graph& graph)
{

    _edges = EdgeIDMap();
    _vertices = VertexIDMap();

    Graph::VertexIDMap vertices = graph.vertices();
    for (Graph::VertexIDMap::iterator itV  = vertices.begin(); itV != vertices.end(); itV ++){
        Graph::Vertex* v = itV->second;
        int id = v->id();
        Vector2f position = v->position();
        addVertex(id, position);
    }

    Graph::EdgeIDMap edges = graph.edges();
    for (Graph::EdgeIDMap::iterator itE  = edges.begin(); itE != edges.end(); itE ++){
        Graph::Edge* e = itE->second;
        int id = e->id();
        int fromId = e->from()->id();
        int toId = e->to()->id();
        bool undirected = e->undirected();
        float cost = e->cost();
        int capacity = e->capacity();

        Graph::Edge* _e = addEdge(fromId, toId, undirected, cost, capacity);
        _e->setParentId(e->parentId());

    }

}
Graph & Graph::operator=(const Graph & graph)
{

    if (this == &graph){
        return *this;
    }

    clear();

    Graph::VertexIDMap vertices = graph.vertices();
    for (Graph::VertexIDMap::iterator itV  = vertices.begin(); itV != vertices.end(); itV ++){
        Graph::Vertex* v = itV->second;
        int id = v->id();
        Vector2f position = v->position();
        addVertex(id, position);
    }

    Graph::EdgeIDMap edges = graph.edges();
    for (Graph::EdgeIDMap::iterator itE  = edges.begin(); itE != edges.end(); itE ++){
        Graph::Edge* e = itE->second;
        int id = e->id();
        int fromId = e->from()->id();
        int toId = e->to()->id();
        bool undirected = e->undirected();
        float cost = e->cost();
        int capacity = e->capacity();

        Graph::Edge* _e = addEdge(fromId, toId, undirected, cost, capacity);
        _e->setParentId(e->parentId());

    }

    return *this;


}



void Graph::printVerticesInfo()
{
    for (Graph::VertexIDMap::const_iterator it = _vertices.begin(); it != _vertices.end(); it++){
        Graph::Vertex* v = it->second;
        std::cout<< "Vertex " << v->id() << " at " << v->position().transpose();
        if (v->enteringEdges().size() > 0){
            std::cout <<" +1: ";
            for (Graph::Edge* e : v->enteringEdges()){
                std::cout<<" e " << e->from()<< " " << e->to();
            }
        }
        if (v->exitingEdges().size() > 0){
            std::cout <<" -1: ";
            for (Graph::Edge* e : v->exitingEdges()){
                std::cout<<" e " << e->from()<< " " << e->to();
            }
        }
        std::cout<<std::endl;
    }
}


void Graph::printEdgesInfo()
{
    for (Graph::EdgeIDMap::const_iterator it = _edges.begin(); it != _edges.end(); it++){
        Graph::Edge* e = it->second;
        std::cout<< "Edge " << e->from()->id() << " "<< e->to()->id() << (e->undirected() ? " Undirected" : " Directed") << " Cost: " << e->cost() << " Capacity: " << (e->capacity() == INT_MAX ? std::numeric_limits<float>::infinity() : e->capacity()) << " Id " << e->id();
        if (e->parentId() != e->id()){
            std::cout << " ParentId: " << e->parentId();
        }
        std::cout<<std::endl;
    }
}
