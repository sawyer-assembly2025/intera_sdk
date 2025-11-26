# -*- coding: utf-8 -*-
"""
Author:
    Hector Quijada

Fuzzy ARTMAP implemetation using ARTa, ARTb, and Map Field

Nodes at ARTa and ARTb increase and dimensions at Map Field are adjusted accordingly

G. A. Carpenter, S. Grossberg, N. Markuzon, J. H. Reynolds and D. B. Rosen, "Fuzzy ARTMAP: A neural network architecture 
for incremental supervised learning of analog multidimensional maps," in IEEE Transactions on Neural Networks, vol. 3, 
no. 5, pp. 698-713, Sept. 1992, doi: 10.1109/72.159059. keywords: {Fuzzy neural networks;Neural networks;Fuzzy logic;
Resonance;Subspace constraints;Computational modeling;Fuzzy systems;Supervised learning;Multidimensional systems;Fuzzy sets},


"""
#Import libraries

import numpy as np
import time

class FuzzyArtMap:
    
    def __init__(self, ARTa_nodes = 1, ARTb_nodes = 1, baseline_vigilance=0.9):
        
        #Initial number of nodes
        self.ARTa_nodes = ARTa_nodes
        self.ARTb_nodes = ARTb_nodes
        
        #Vigilance Parameters
        self.rho_a = baseline_vigilance
        self.rho_b = 0.9
        self.rho_ab = 0.95
        
        #Learning Rates
        self.beta_a = 1.0
        self.beta_b = 1.0
        self.beta_ab = 1.0
        
        #Commited Learning Rates
        self.committed_beta_a = 0.75
        self.committed_beta_b = 0.75
        self.committed_beta_ab = 0.75
        
        #Choice Parameter
        self.alpha = 0.001
        
        #Commited nodes
        self.committed_nodes = set()
        
        #Reset nodes
        self.reset_nodes = []
        
        #Resonance Flag
        self.resonance = False
        
        #Match Tracking increase
        self.epsilon = 0.001
        
        #Epochs Variable
        self.epochs = 0
        
        #Weights Variable for prediction
        self.weight_a = 0
        self.weight_b = 0
        self.weight_ab = 0
    
    #Extract inputs and outputs relation in a csv file, give dimensions for each input for training purposes
    def _extract_csv_inputs(self, Ia_dim, Ib_dim, path):
        
        #Initialize Auxiliary Inputs
        Ia_aux = np.zeros(Ia_dim)
        Ib_aux = np.zeros(Ib_dim)
        
        #Initialize inputs as numpy array
        Ia = np.zeros(Ia_dim)
        Ib = np.zeros(Ib_dim)
        
        #Extract all Inputs from train file as a single numpy array for ARTa and ARTb
        with open(path, 'r') as file:
            
            # Read all lines and process each one
            lines = file.readlines()
            for line in lines:
                inputs = line.strip().split(',') #Extract EOL and split in a list with separator ","
                
                #Split into two vectors and convert to float
                for i in range(Ia_dim):
                    Ia_aux[i] = float(inputs[i]) 
                
                for i in range(Ib_dim):
                    Ib_aux[i] = float(inputs[Ia_dim+i])
                
                #Stack current line vector into a matrix
                Ia = np.vstack((Ia, Ia_aux))
                Ib = np.vstack((Ib, Ib_aux))
            
            #Delete first row as it was auxiliary
            Ia = np.delete(Ia, 0, axis=0)
            Ib = np.delete(Ib, 0, axis=0)
        
        return Ia, Ib
    
    #Extract weights of a generated training csv file, for prediction purposes
    def _extract_csv_weight(self, w_dim, weights_path):

        w = np.zeros(w_dim)
        w_aux = np.zeros(w_dim)

        #Extract all Inputs from train file as a single numpy array for ARTa and ARTb
        with open(weights_path, 'r') as file:
            
            # Read all lines and process each one
            lines = file.readlines()
            for line in lines:
                w_str = line.strip().split(',') #Extract EOL and split in a list with separator ","
                
                #Split into two vectors and convert to floar
                for i in range(len(w_str)):
                    w_aux[i] = float(w_str[i])
                
                #Stack current line vector into a matrix
                w = np.vstack((w, w_aux))
                #Delete first row as it was auxiliary
            
            w = np.delete(w, 0, axis=0)
        
        return w
    
    #Save weights results from training
    def _save_csv_weights(self, weight_a, weight_b, weight_ab):
        
        weights_a_path = "weights_a.csv"
        weights_b_path = "weights_b.csv"
        weights_ab_path = "weights_ab.csv"
        np.savetxt(weights_a_path, weight_a, delimiter=',', fmt='%f')
        np.savetxt(weights_b_path, weight_b, delimiter=',', fmt='%f')
        np.savetxt(weights_ab_path, weight_ab, delimiter=',', fmt='%f')
    
    #Add complement to input
    def _complement_encode(self,Ia,Ib):
        
        complement_encoded_Ia = np.zeros(Ia.shape[1]*2)
        complement_encoded_Ib = np.zeros(Ib.shape[1]*2)
        
        #Rearrange Inputs with complements
        for i in range(Ia.shape[0]):
            complement_encoded_Ia = np.vstack((complement_encoded_Ia,np.concatenate((Ia[i], 1-Ia[i]))))
            complement_encoded_Ib = np.vstack((complement_encoded_Ib,np.concatenate((Ib[i], 1-Ib[i]))))
                
        complement_encoded_Ia = np.delete(complement_encoded_Ia, 0, axis=0)
        complement_encoded_Ib = np.delete(complement_encoded_Ib, 0, axis=0)
        
        return complement_encoded_Ia, complement_encoded_Ib
    
    #Search for input resonance given a set of nodes
    def _resonance_search(self,weight,Ii,rho):
        
        #Reset nodes
        self.reset_nodes = []
        
        #Resonance Flag
        self.resonance = False
        
        #Initialize T
        T = np.zeros((weight.shape[0]))

        #Obtain activation value for each node at ARTa
        for j in range(weight.shape[0]):
            category_choice_numerator = np.sum(np.minimum(Ii, weight[j]))
            category_choice_denominator = self.alpha + np.sum(weight[j])
            category_choice = category_choice_numerator / category_choice_denominator
            T[j] = category_choice
        
        while not self.resonance:
            
            #If node already reset set activation value to 0 to avoid it competing
            if len(self.reset_nodes) > 0:
                for reset in self.reset_nodes:
                    T[reset] = 0
            
            #Obtain max activation value
            J = np.argmax(T) #If more than 1 max value first index is implicitely selected
            
            #Compute current vigilance value to compare with criteria rho_a
            vigilance_criteria_numerator = np.sum(np.minimum(Ii, weight[J]))
            vigilance_criteria_denominator = np.sum(Ii)
            vigilance_criteria = vigilance_criteria_numerator/vigilance_criteria_denominator
            
            if vigilance_criteria >= rho:
                self.resonance = True
            else:
                if len(self.reset_nodes) < len(T):
                    self.reset_nodes.append(J)
                else:
                    #We break while loop and resonance in Fuzzy ART remains false to trigger learning of new
                    #created node
                    
                    #Add new node to learn new pattern
                    weight = np.vstack((weight, np.ones((1,Ii.shape[0]))))
                    
                    J = len(T)
                    break
        
        return J, self.resonance, vigilance_criteria, weight
    
    #Train Fuzzy ARTMAP network with a set of input, outputs relations
    def train(self, Ia_dim, Ib_dim, train_path, save_weights=True):
        
        #Extract inputs from csv 
        Ia, Ib = self._extract_csv_inputs(Ia_dim, Ib_dim, train_path)
        
        #Add complement encoding to original inputs
        Iac, Ibc = self._complement_encode(Ia, Ib)
        
        #Initialize all weights to 1
        weight_a = np.ones((self.ARTa_nodes, Ia_dim*2))
        weight_b = np.ones((self.ARTb_nodes, Ib_dim*2))
        weight_ab = np.ones((self.ARTa_nodes, self.ARTb_nodes))
        
        #Loop Training for each set of inputs Ia, Ib (Must be same number of inputs)
        for i in range(Ia.shape[0]):
            
            #Resonance Flags(ARTa, ARTb, Map Field)
            resonance_a = False
            resonance_b = False
            resonance_ab = False
            
            #ARTa Resonance Search with baseline vigilance
            Ja, resonance_a, rho_a, weight_a = self._resonance_search(weight=weight_a, Ii=Iac[i], rho=self.rho_a)
            #ARTb Resonance Search
            Jb, resonance_b, rho_b, weight_b = self._resonance_search(weight=weight_b, Ii=Ibc[i], rho=self.rho_b)
            
            if resonance_a and not resonance_b:
                weight_ab = np.hstack((weight_ab, np.zeros((self.ARTa_nodes,1))))
                self.ARTb_nodes += 1
                weight_ab = np.vstack((weight_ab, np.ones((1,self.ARTb_nodes))))
                self.ARTa_nodes += 1

            else:
                #Update weights at mapfield
                if not resonance_b:
                    weight_ab = np.hstack((weight_ab, np.zeros((self.ARTa_nodes,1))))
                    self.ARTb_nodes += 1
                if not resonance_a:
                    weight_ab = np.vstack((weight_ab, np.ones((1,self.ARTb_nodes))))
                    self.ARTa_nodes += 1
            
            #Match Tracking at map field Fab
            while not resonance_ab:
                
                #Initialize y_b node
                y_b = np.zeros(self.ARTb_nodes)
                y_b[Jb] = 1
                
                #Evaluate if the nodes at F2a and F2b have been activated
                if resonance_a == True and resonance_b == True:
                    x_ab = np.minimum(y_b, weight_ab[Ja]) 
                elif resonance_a == True and resonance_b == False:
                    x_ab = np.sum(weight_ab[Ja])
                    
                    #ARTb output disregarded ARTa output, that is because we did not found the output in current
                    #weights at ARTb, but we did at ARTa, that is a contradiction, since we are doing a relation
                    #between inputs and outputs, that is as saying I found something that does not exist yet
                    #that is why you need to consider the node at ARTb as the parent for this weight update and also
                    #add a new neuron at ARTa, since we know that we already know the prediction is not correct
                    Ja = Jb
                    
                    #Do not add weight at last input
                    if i != (Ia.shape[0]-1):
                        weight_a = np.vstack((weight_a, np.ones((1,Ia_dim*2))))
                        
                elif resonance_a == False and resonance_b == True:
                    x_ab = y_b
                elif resonance_a == False and resonance_b == False:
                    x_ab = 0.0
                    
                #Compute current vigilance value to compare with criteria rho_ab
                map_vigilance_criteria = np.sum(x_ab)/np.sum(y_b)
                
                if map_vigilance_criteria >= self.rho_ab:
                    
                    #Update learning rate if nodes already commited
                    
                    beta_b = self.beta_b
                    
                    if Ja in self.committed_nodes:
                        beta_a = self.committed_beta_a
                        beta_ab = self.committed_beta_ab
                    else:
                        beta_a = self.beta_a
                        beta_ab = self.beta_ab
                    
                    #Update weights based on learning law
                    weight_a[Ja] = (beta_a * (np.minimum(Iac[i],weight_a[Ja]))) + ((1 - beta_a) * weight_a[Ja])
                    weight_b[Jb] = (beta_b * (np.minimum(Ibc[i],weight_b[Jb]))) + ((1 - beta_b) * weight_b[Jb])
                    weight_ab[Ja] = (beta_ab * (np.minimum(y_b,weight_ab[Ja]))) + ((1 - beta_ab) * weight_ab[Ja])
                    self.committed_nodes.add(Ja)
                    
                    resonance_ab = True

                else:
                    #If map field vigilance parameter condition not met increase rho_a
                    self.reset_nodes.append(Ja)
                    
                    rho_a += self.epsilon
                    
                    Ja, resonance_a, rho_a, weight_a = self._resonance_search(weight=weight_a, Ii=Iac[i], rho=rho_a)
            
        #Save weights
        if save_weights:
            self._save_csv_weights(weight_a, weight_b, weight_ab)
        
        return weight_a.shape[1], weight_b.shape[1], weight_ab.shape[1]
    
    #Load trained weights once in a prediction loop
    def load_weights(self, wa_dim, wb_dim, wab_dim):
        
        weights_a_path = "weights_a.csv"
        self.weight_a = self._extract_csv_weight(w_dim=wa_dim, weights_path=weights_a_path)
        weights_b_path = "weights_b.csv"
        self.weight_b = self._extract_csv_weight(w_dim=wb_dim, weights_path=weights_b_path)
        weights_ab_path = "weights_ab.csv"
        self.weight_ab = self._extract_csv_weight(w_dim=wab_dim, weights_path=weights_ab_path)
    
    #For testing purposes, create a csv file and extract inputs
    def extract_csv_input(self, Ia_dim, path):
        
        #Initialize Auxiliary Inputs
        Ia_aux = np.zeros(Ia_dim)
        
        #Initialize inputs as numpy array
        Ia = np.zeros(Ia_dim)
        
        #Extract all Inputs from train file as a single numpy array for ARTa and ARTb
        with open(path, 'r') as file:
            
            # Read all lines and process each one
            lines = file.readlines()
            for line in lines:
                inputs = line.strip().split(',') #Extract EOL and split in a list with separator ","
                
                #Split into two vectors and convert to float
                for i in range(Ia_dim):
                    Ia_aux[i] = float(inputs[i]) 
                
                #Stack current line vector into a matrix
                Ia = np.vstack((Ia, Ia_aux))
            
            #Delete first row as it was auxiliary
            Ia = np.delete(Ia, 0, axis=0)
        
        complement_encoded_Ia = np.zeros(Ia.shape[1]*2)
        
        #Rearrange Inputs with complements
        for i in range(Ia.shape[0]):
            complement_encoded_Ia = np.vstack((complement_encoded_Ia,np.concatenate((Ia[i], 1-Ia[i]))))
                
        complement_encoded_Ia = np.delete(complement_encoded_Ia, 0, axis=0)
        
        return complement_encoded_Ia
    
    #Predict given a set of trained weights
    def predict(self,Ia):
    
        #Resonance Flags(ARTa, ARTb, Map Field)
        resonance_a = False
        
        #ARTa Resonance Search with baseline vigilance
        Ja, resonance_a, rho_a, self.weight_a = self._resonance_search(weight=self.weight_a, Ii=Ia, rho=self.rho_a)
        
        #Update ARTa neuron number
        if not resonance_a:
            self.ARTa_nodes += 1
                
        return Ja
        

if __name__ == "__main__":
  
    artmap = FuzzyArtMap()
    
    #Train Fuzzy ARTMAP (Give dimensions before adding complement)
    wa_dim, wb_dim, wab_dim = artmap.train(Ia_dim=6, Ib_dim=4, train_path="train.csv", save_weights=True)
    
    #Load trained weights
    artmap.load_weights(wa_dim, wb_dim, wab_dim)
    
    #Extract test inputs
    I = artmap.extract_csv_input(Ia_dim=6, path="test.csv")
    
    for i in range(I.shape[0]):
        start_time = time.perf_counter()
        category_predicted = artmap.predict(I[i])
        print(f"Input {i+1} is clustered at neuron: {category_predicted}")
        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.6f} seconds")
        

    
    
    