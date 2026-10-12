# Why the AGPLv3 license?

If you are wondering why I chose to license my custom firmware as AGPLv3, the answer is actually quite simple. If you don't know, the AGPLv3 and GPLv3 are almost identical licenses with one very important difference: Article 13. (Which, by the way, if you haven't, I encourage you to read both licenses, even though it's very much a lot of legalese, it does help you understand the background).

Article 13 of the GNU General Public License makes the AGPLv3 explicitly a compatible license and also explicitly turns a combination of GPLv3 and AGPLv3 licensed things into an AGPLv3 combination. Which means the Jentastic firmware, when compiled with the Nexus module (otherwise it wouldn't make any sense as a firmware without it), is treated as entirely AGPLv3.

Article 13 of the AGPLv3 closes, as some people call it, the private fork loophole, as the GPLv3 only triggers its protection on distribution, and not network-accessed use, as AGPLv3's Article 13 includes.
Quoting the license literally, it says: "Notwithstanding any other provision of this License, if you modify the Program, your modified version must prominently offer all users interacting with it remotely through a computer network (if your version supports such interaction) an opportunity to receive the Corresponding Source of your version by providing access to the Corresponding Source from a network server at no charge, through some standard or customary means of facilitating copying of software."
I am of the opinion, although I'm not a lawyer, that Meshtastic nodes, especially infrastructure nodes, which are computing devices by the fact they have an MCU and computer software, and interact with the mesh by routing packets, responding to requests (nodeinfos, telemetry, traceroutes and whatnot), would count as interacting with it through a computer network.

So this firmware is made for the different Meshtastic communities, with the Spanish community in mind but not necessarily just that, and I believe these collaborative, volunteer-owned networks should be maximally collaborative. You can't get to the stage the Spanish mesh is right now with more than 2000 active nodes all in one RF-only mesh covering most of the mainland without this teamwork and collaboration.

Even though it probably sounds a bit hypocritical as this firmware was developed for around a year as a private fork shared with trusted people (starting around 2.7.10 if I remember correctly and forwards), for any derivative work of Jentastic, I want the full teeth of the AGPLv3 because I believe it's important for the overall mesh and community at large.

I could have done the same with my changes to upstream code as nothing prevents that, GPLv3 explicitly allows it even as you see. But I deliberately left changes to upstream code as GPLv3 because that makes it way easier to make future contributions to the upstream firmware, which I plan on still doing, obviously. So because I think it's extremely unlikely the upstream firmware decides to relicense to AGPLv3 having those parts of the code as GPLv3 lets me not shoot myself in the foot in that way.

I hope this explains somewhat the license choice.🙂

