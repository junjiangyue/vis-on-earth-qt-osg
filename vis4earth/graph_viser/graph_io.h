#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_LOADER_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_LOADER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <vis4earth/graph_viser/graph.h>

namespace VIS4Earth {

class GraphLoader {
  private:
    FILE *filePointer;
    int fileState;
    std::string fileName;

    enum MK_FILE_STATE { MK_EMPTY, MK_WRITE, MK_READ };
    void perror(const char *errorMessage) const {
        if (fileName == "_UNDEFINED_")
            printf("%s\n", errorMessage);
        else
            printf("%s (\"%s\").\n", errorMessage, fileName.c_str());

        if (filePointer != NULL)
            fclose(filePointer);

        exit(0);
    };
    void CheckState(const char *functionName, MK_FILE_STATE requiredState) const {
        if (fileState == requiredState)
            return;
        else {
            std::string err_msg = std::string(functionName) + " error: ";
            switch (fileState) {
            case MK_EMPTY: {
                err_msg += "file is not open";
                break;
            }
            case MK_WRITE: {
                err_msg += "file is open for writing";
                break;
            }
            case MK_READ: {
                err_msg += "file is open for reading";
                break;
            }
            }
            perror(err_msg.c_str());
        }
    };

  public:
    GraphLoader() {
        filePointer = NULL;
        fileState = MK_EMPTY;
        fileName = "_UNDEFINED_";
    }
    GraphLoader(const std::string &myFileName, const char *mode) {
        // set file name
        fileName = std::string(myFileName);

        // set state
        fileState = MK_READ;
        if (strcmp(mode, "w") == 0)
            fileState = MK_WRITE;

        // try to Open depending on the state
        filePointer = NULL;
        switch (fileState) {
        case MK_READ: {
            filePointer = fopen(fileName.c_str(), "r");
            break;
        }
        case MK_WRITE: {
            filePointer = fopen(fileName.c_str(), "w");
            break;
        }
        }

        // if failed, print error
        if (filePointer == NULL)
            perror("constructor error: file not found");
    };
    ~GraphLoader() {
        if (fileState != MK_EMPTY && filePointer != NULL)
            fclose(filePointer);
    }
    bool Open(const std::string &myFileName, const char *mode) {
        // set state
        fileState = MK_READ;
        if (strcmp(mode, "w") == 0)
            fileState = MK_WRITE;

        // try to Open depending on the file state
        filePointer = NULL;
        switch (fileState) {
        case MK_READ: {
            filePointer = fopen(myFileName.c_str(), "r");
            break;
        }
        case MK_WRITE: {
            filePointer = fopen(myFileName.c_str(), "w");
            break;
        }
        }

        // if failed, print message and exit
        if (filePointer == NULL)
            return false;
        else
            return true;
    }
    void Close() {
        switch (fileState) {
        case MK_EMPTY:
            perror("close error: file is not opened");

        case MK_READ:
        case MK_WRITE: {
            if (filePointer != NULL)
                fclose(filePointer);
            fileState = MK_EMPTY;
            fileName = "_UNDEFINED_";
        }
        }
    };
    void GoToTop() {
        CheckState("GoToTop", MK_READ);
        fseek(filePointer, 0, SEEK_SET);
    };
    int Rows() const {
        CheckState("rows", MK_READ);
        char line[1024] = {""};
        int _rows = 0;
        fseek(filePointer, 0, SEEK_SET);
        while (!feof(filePointer)) {
            fgets(line, sizeof(line), filePointer);
            _rows++;
        }
        fseek(filePointer, 0, SEEK_SET);

        return _rows - 1;
    };
    int Columns() const {
        CheckState("columns", MK_READ);

        char line[1024] = {""};
        char *line_ptr = fgets(line, sizeof(line), filePointer);
        char col[1024] = {""};
        int _columns = 0, shift = 0;
        fseek(filePointer, 0, SEEK_SET);
        while (sscanf(line_ptr, "%s%n", col, &shift) == 1) {
            _columns++;
            line_ptr += shift;
        }
        fseek(filePointer, 0, SEEK_SET);

        return _columns;
    };
    void PutData(const std::vector<double> &data) {
        CheckState("PutData", MK_WRITE);

        int len = (int)data.size();
        fprintf(filePointer, "%lg", data[0]);
        for (int i = 1; i < len; i++)
            fprintf(filePointer, " %lg", data[i]);
        fprintf(filePointer, "\n");
    };
    void PutText(char *text) {
        CheckState("PutText", MK_WRITE);
        fprintf(filePointer, "%s\n", text);
    };
    int GetData(std::vector<double> &data) {
        CheckState("GetData", MK_READ);

        int num_data_element = 0;
        int shift = 0;
        double data_element = 0.0;
        char *data_line, line[1024] = {""};

        data.clear();
        data_line = fgets(line, sizeof(line), filePointer);
        while (sscanf(data_line, "%lf%n", &data_element, &shift) == 1) {
            data.push_back(data_element);
            num_data_element++;
            data_line += shift;
        }

        return num_data_element;
    };
    void GetText(char *text, int length) {
        CheckState("GetText", MK_READ);

        fgets(text, length - 1, filePointer);
        text[strlen(text) - 1] = '\0';
    };

    // read csv file to graph
    static VIS4Earth::Graph LoadFromFile(const std::string &nodesFile,
                                         const std::string &edgesFile) {
        VIS4Earth::GraphLoader f;
        int rows = 0;
        char line[1024] = {""};

        // read nodes
        if (!f.Open(nodesFile, "r")) {
            exit(0);
        }
        rows = f.Rows();
        f.GetText(line, 1024);
        int labelCh;
        char name[100];
        double latitude, longitude;
        std::unordered_map<std::string, Node> read_nodes;

        for (int r = 1; r < rows; r++) {
            f.GetText(line, 1024);
            sscanf(line, "%d,%[^,],%lg,%lg", &labelCh, name, &latitude, &longitude);
            read_nodes.insert(
                std::pair<std::string, Node>(std::to_string(labelCh), Node(latitude, longitude)));
        }
        f.Close();

        // read edgeFile
        VIS4Earth::GraphLoader edgeF;
        int rowsEdge = 0;
        char lineEdge[1024] = {""};
        if (!edgeF.Open(edgesFile, "r")) {
            exit(0);
        }
        rowsEdge = edgeF.Rows();
        edgeF.GetText(lineEdge, 1024);
        char src[128], dst[128];
        double w, wmax = 0.0;
        std::vector<Edge> allEdges;
        for (int r = 0; r < rowsEdge - 1; r++) {
            edgeF.GetText(line, 1024);
            w = 1.0;
            sscanf(line, "%s %s %lg", src, dst, &w);
            allEdges.push_back(Edge(std::string(src), std::string(dst),
                                    read_nodes[std::string(src)].pos,
                                    read_nodes[std::string(dst)].pos, w + 1.0));
        }
        edgeF.Close();

        VIS4Earth::Graph graph;
        graph.set(read_nodes, allEdges);
        graph.calculateGeoNodeLevel();
        return graph;
    }
    static VIS4Earth::Graph LoadFromNoGeoFile(const std::string &nodesFile,
                                              const std::string &edgesFile) {
        std::srand(std::time(0));
        VIS4Earth::GraphLoader f;
        int rows = 0;
        char line[1024] = {""};

        // read nodes
        if (!f.Open(nodesFile, "r")) {
            exit(0);
        }
        rows = f.Rows();
        f.GetText(line, 1024);
        int labelCh;
        char name[100];
        double latitude, longitude;
        std::unordered_map<std::string, Node> read_nodes;

        for (int r = 1; r < rows; r++) {
            f.GetText(line, 1024);
            sscanf(line, "%d,%[^,]", &labelCh, name);
            float x = std::rand() % 100;
            float y = std::rand() % 100;
            read_nodes.insert(std::pair<std::string, Node>(std::to_string(labelCh), Node(x, y)));
        }
        f.Close();

        // read edgeFile
        VIS4Earth::GraphLoader edgeF;
        int rowsEdge = 0;
        char lineEdge[1024] = {""};
        if (!edgeF.Open(edgesFile, "r")) {
            exit(0);
        }
        rowsEdge = edgeF.Rows();
        edgeF.GetText(lineEdge, 1024);
        char src[128], dst[128];
        double w, wmax = 0.0;
        std::vector<Edge> allEdges;
        for (int r = 0; r < rowsEdge - 1; r++) {
            edgeF.GetText(line, 1024);
            w = 1.0;
            sscanf(line, "%s %s %lg", src, dst, &w);
            allEdges.push_back(Edge(std::string(src), std::string(dst),
                                    read_nodes[std::string(src)].pos,
                                    read_nodes[std::string(dst)].pos, w + 1.0));
        }
        edgeF.Close();

        VIS4Earth::Graph graph;
        graph.set(read_nodes, allEdges);
        graph.calculateNoGeoNodeLevel();
        graph.calculateNoGeoEdgeWeight();
        return graph;
    }
};

} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_GRAPH_LOADER_H