#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

class Ponto{
    float _x, _z;
  public:
    Ponto(float x = 0, float z = 0) : _x(x), _z(z) {};
    float getX() const { return _x;}
    float getZ() const { return _z;}
};

class Trajetoria
{
private:
	std::vector<Ponto> pontos; 
public:
	Trajetoria(){
	}

	void save(std::string filename) const {
		std::ofstream file(filename);
		for (int i = 0; i<pontos.size(); i++)
			file << pontos[i].getX() << ";" << pontos[i].getZ() << std::endl;
		file.close();
	}

	void load(std::string filename) {
		std::ifstream file(filename);
		std::string linha, coluna;
		float ponto[2];
		while (!file.eof()) {
			std::getline(file, linha);
			std::istringstream Colunas(linha);
			while (!Colunas.eof()) {
				for (int i = 0; i<2; i++) {
					std::getline(Colunas, coluna, ';');
					if (coluna != "")
						ponto[i] = atof(coluna.c_str());
				}
				addPoint(ponto[0], ponto[1]);
			}
		}
		file.close();
	}

	void addPoint(float x, float z) {
		Ponto ponto(x, z);
		pontos.push_back(ponto);
	}

	Ponto getPoint(int indice) const {
		if (indice >= 0 && indice < pontos.size())
			return pontos[indice];
		else
			throw std::out_of_range("waypoint index out of range");
	}
	
	bool distanceGreaterThan1m(float x, float z) {
  	       if (std::isnan(x) || std::isnan(z))
    	               return false;
	       if (pontos.size() > 0) {
    	               float xn = pontos[pontos.size()-1].getX();
    	               float zn = pontos[pontos.size()-1].getZ();
      	               float distance = std::sqrt( std::pow(xn-x,2.0) + std::pow(zn-z,2.0) );
      	               if (distance > 1.0)
      	                      return true;
      	               else
        	               return false;
      	       }
      	       else
        	       return true;
      	}

};