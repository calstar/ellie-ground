
close all
figure
hold on
test1=		[	10735	10751	10760	10767	10773	10778	10783	10787	10791	10797	10804	10812	10820	10828	10837	10845	10853	10861	10870	10878	10886	10894	10902	10911	10919	10927	10935	10943	10952	10960	10968	10976	10984	10992	11001	11009	11017	11025	11033	11041	11049	11058	11066	11074	11082	11091	11099	11107	11115	11124	11132	11140	11148	11157	11165	11173	11181	11190	11198	11206	11215	11223	11231	11240	11249	11258	11267	11276	11286	11298	11315	11358		]															
test2= [			12734	12749	12757	12764	12770	12775	12780	12785	12793	12801	12810	12818	12827	12835	12844	12852	12860	12869	12877	12885	12894	12902	12910	12919	12927	12936	12944	12952	12961	12969	12977	12986	12994	13003	13011	13020	13028	13037	13045	13053	13062	13070	13078	13087	13095	13104	13112	13120	13129	13137	13145	13154	13162	13171	13179	13188	13196	13204	13213	13222	13230	13239	13248	13257	13267	13276	13288	13303	13337																				]
test3= [			20042	20059	20069	20076	20083	20088	20093	20099	20108	20116	20125	20134	20143	20151	20160	20169	20177	20186	20194	20203	20211	20220	20228	20237	20246	20254	20263	20271	20280	20288	20297	20305	20314	20323	20331	20340	20348	20357	20365	20374	20382	20391	20400	20408	20417	20426	20434	20443	20452	20460	20469	20478	20486	20495	20504	20512	20521	20529	20538	20547	20556	20564	20573	20582	20591	20600	20609	20617	20626	20635	20644	20652	20661	20670	20679	20688	20696	20705	20714	20723	20733	20742	20751	20760	20770	20780	20791	20805	20836]



df1=diff(test1)

plot((test1(1:end-1)-test1(1))/1000,smooth(1./df1)*.076*1000 )

 


df2=diff(test2)


plot((test2(1:end-1)-test2(1))/1000,smooth(1./df2)*.076*1000 )



df3=diff(test3)


plot((test3(1:end-1)-test3(1))/1000,smooth(1./df3)*.076*1000 )

legend('Initial 101 psi','Initial 115 psi','Initial 147 psi')
xlabel('Time [s]')
ylabel('Mass Flow Rate [g/s]')

