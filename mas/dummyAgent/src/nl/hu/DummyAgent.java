package nl.hu;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.TickerBehaviour;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.io.IOException;
import java.util.ArrayList;

import nl.hu.blackboard.client.BlackboardClientUtils;
import nl.hu.blackboard.data.PostItProtos.PostIt;
import nl.hu.blackboard.data.PostItProtos.PostItBox;
import nl.hu.autokeystore.clientlib.*;

public class DummyAgent extends Agent 
{
	private AutoKeyStore autoKeyStore;
	private String blackboardIp;
	private int blackboardPort;
	private int ordinal = 1;
	public void setup()
	{
		autoKeyStore = new AutoKeyStore();
		boolean discovering = true;
		// try to discover the autoKeyStoreServer

		while(discovering)
		{
			try
			{
				System.out.println("Looking for autokeyStoreServer");
				blackboardIp = autoKeyStore.getValue("blackboard.ip");
				blackboardPort = Integer.parseInt(autoKeyStore.getValue("blackboard.port"));
				discovering = false;
				System.out.println("got blackboard at " + blackboardIp+":"+blackboardPort);
			}
			catch(Exception e)
			{
				System.out.println("No autoKeyStoreServer found :O");
			}

	     

					 			   
			


		}

		this.addBehaviour(new CyclicBehaviour()
		{
			@Override
			public void action() 
			{
				ACLMessage message = myAgent.blockingReceive();
				ArrayList<PostIt> arr = new ArrayList<PostIt>();	
				PostIt p = PostIt.newBuilder().setOrdinal(ordinal).setOwner("DummyAgent").setPayload("PayLoad").build();			 			   
				arr.add(p);
		    	PostItBox postItBox = PostItBox.newBuilder().setZone("BB1").setIsWrite(true).addAllPostIts(arr)
			    .build();
			    BlackboardClientUtils.writeToBlackboard(postItBox);	
			    ordinal++;
				
			}
		});
		



	}
	

}
